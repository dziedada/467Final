#include <vision/core/utilities.hpp>
#include <vision/core/RealsenseInterface.hpp>
#include <vision/core/BallDetector.hpp>
#include <vision/core/CloudTransformer.hpp>
#include <common/message_channels.hpp>
#include <common/messages/depth_image_t.hpp>
#include <common/messages/rgb_image_t.hpp>
#include <common/messages/ball_detections_t.hpp>

#include <opencv2/core.hpp>
#include <zcm/zcm-cpp.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <functional>

using std::cout;
using std::endl;
using std::cerr;
using std::string;
using std::vector;
using std::shared_ptr;
using std::thread;
using std::mutex;
using std::unique_lock;
using std::condition_variable;
using std::atomic_bool;
using std::function;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::duration_cast;
using std::chrono::system_clock;

using zcm::ZCM;

using cv::Mat;

using pcl::PointCloud;
using pcl::PointXYZ;

using vision::CameraPoseData;
using vision::RealsenseInterface;

// Start MeasureFramerate Section **********************************

class MeasureFramerate
{
public:
    void add_sample(int64_t sample);
private:
    static void compute_statistics(MeasureFramerate* self);
    static double compute_average_disparity(const vector<int64_t>& disparities, size_t start,
        size_t end);
    vector<int64_t> timestamps_;
    mutex mtx_;
};

void MeasureFramerate::add_sample(int64_t sample)
{
    unique_lock<mutex> lck(mtx_);
    timestamps_.push_back(sample);
    // Compute statistics if enough samples have been collected
    if (timestamps_.size() >= 600) thread(compute_statistics, this).detach();
}

void MeasureFramerate::compute_statistics(MeasureFramerate* self)
{
    // Make a local copy of the timestamps
    unique_lock<mutex> lck(self->mtx_);
    vector<int64_t> timestamps = self->timestamps_;
    lck.unlock();
    // Sort the timestamps
    std::sort(timestamps.begin(), timestamps.end());
    // Compute the disparity between timestamps
    vector<int64_t> disparities;
    disparities.reserve(timestamps.size());
    for (size_t i = 1; i < timestamps.size(); ++i)
    {
        disparities.push_back(timestamps[i] - timestamps[i - 1]);
    }
    // Sort the timestamp disparities
    std::sort(disparities.begin(), disparities.end());
    // Compute the average disparity for the largest 1/2 of the disparities
    double largest_half = compute_average_disparity(disparities, disparities.size() / 2, 
        disparities.size());
    // Compute the average disparity for the smallest 1/2 of the disparities
    double smallest_half = compute_average_disparity(disparities, 0, 
        disparities.size() / 2);
    // Compute the average disparity for each 10% of the disparities
    const double step = 0.1; // 10%
    vector<double> percentile_averages;
    percentile_averages.reserve(10);
    for (size_t i = 0; i < 10; ++i)
    {
        percentile_averages.push_back(compute_average_disparity(
            disparities, static_cast<size_t>(step * i * disparities.size()),
            static_cast<size_t>((step * i + 0.1) * disparities.size())));
    }
    // Print the results of the analysis
    cout << "Average Disparity of Largest Half: " << largest_half << endl;
    cout << "Average Disparity of Smallest Half: " << smallest_half << endl;
    for (size_t i = 0; i < percentile_averages.size(); ++i)
    {
        size_t lower_percent = (i * 10);
        size_t uppper_percent = lower_percent + 10;
        cout << lower_percent << " to " << uppper_percent << ": " << percentile_averages[i] << endl;
    }
    cout << "Average FPS equivalent of Largest Half: " << 1000000.0 / largest_half << endl;
    cout << "Average FPS equivalent of Smallest Half: " << 1000000.0 / smallest_half << endl;
    for (size_t i = 0; i < percentile_averages.size(); ++i)
    {
        size_t lower_percent = (i * 10);
        size_t uppper_percent = lower_percent + 10;
        cout << lower_percent << " to " << uppper_percent << ": " 
            << 1000000.0 / percentile_averages[i] << endl;
    }
}

double MeasureFramerate::compute_average_disparity(const vector<int64_t>& disparities, 
    size_t start, size_t end)
{
    int64_t disparity_accumulator = 0;
    for (size_t i = start; i < end; ++i)
    {
        disparity_accumulator += disparities[i];
    }
    double average_disparity = static_cast<double>(disparity_accumulator) / 
        static_cast<double>(end - start);
    return average_disparity;
}

// End MeasureFramerate Section *************************************

// Start CameraWrapper Section **************************************

class CameraWrapper
{
public:
    CameraWrapper(const YAML::Node& config, const YAML::Node& root_config, shared_ptr<ZCM> zcm_ptr);
    void start();
    void stop();
    void join();
    void init_tasks(); // Set starting task metadata
    static void run_camera(CameraWrapper* self);
    static void broadcast_rgbd(const Mat rgb, const Mat depth, int64_t utime, CameraWrapper* self);
    static void add_framerate_sample(int64_t utime, CameraWrapper* self);
    static void detect_balls(const Mat rgb, const PointCloud<PointXYZ>::Ptr unordered_cloud, 
        const PointCloud<PointXYZ>::Ptr ordered_cloud, int64_t utime, CameraWrapper* self);
    static void calibration(const Mat rgb, const PointCloud<PointXYZ>::Ptr unordered_cloud, 
        const PointCloud<PointXYZ>::Ptr ordered_cloud, CameraWrapper* self);
private:
    RealsenseInterface interface_;
    shared_ptr<ZCM> zcm_ptr_;
    atomic_bool stay_alive_;
    bool enabled_;
    BallDetector ball_detector_;
    CloudTransformer cloud_transformer_;
    thread camera_thread_;
    // Task metadata section (managed)
    atomic_bool broadcast_rgbd_running_;
    atomic_bool ball_detection_running_;
    // End Task metadata section
    static MeasureFramerate measure_framerate_;
};

CameraWrapper::CameraWrapper(const YAML::Node& config, const YAML::Node& root_config, 
    shared_ptr<ZCM> zcm_ptr) :
    interface_ {RealsenseInterface(config)}, zcm_ptr_ {zcm_ptr}, stay_alive_ {atomic_bool(true)},
    enabled_ {config["enabled"].as<bool>()}, ball_detector_ {BallDetector()}, 
    cloud_transformer_ {CloudTransformer(config)}
{
}

void CameraWrapper::start()
{
    if (!enabled_) return;
    camera_thread_ = thread(run_camera, this);
}

void CameraWrapper::stop()
{
    stay_alive_ = false;
}

void CameraWrapper::join()
{
    if (enabled_) camera_thread_.join();
}

void CameraWrapper::init_tasks()
{
    broadcast_rgbd_running_ = false;
    ball_detection_running_ = false;
}

void CameraWrapper::run_camera(CameraWrapper* self)
{
    // Disable the autoexposure, is here because this has sleeps in it
    // It is better to be done asynchronyously, else it would be in the constructor
    self->interface_.disableAutoExposure();
    self->init_tasks();
    // Read frames and spawn off handlers
    while (self->stay_alive_)
    {
        // Load the next frame, if no new frame then skip iteration
        if (!self->interface_.loadNext()) continue;
        // Grab everything needed for tasks
        cv::Mat rgb = self->interface_.getRGB();
        cv::Mat depth = self->interface_.getDepth();
        PointCloud<PointXYZ>::Ptr unordered_cloud = self->interface_.getPointCloudBasic();
        PointCloud<PointXYZ>::Ptr ordered_cloud = self->interface_.getMappedPointCloud();
        // Align clouds with arm coordinate frame
        unordered_cloud = self->cloud_transformer_.transform(unordered_cloud);
        ordered_cloud = self->cloud_transformer_.transform(ordered_cloud);
        int64_t utime = self->interface_.getUTime();
        // Start up task threads (task threads handle lifetime on their own using task metadata)
        thread(CameraWrapper::broadcast_rgbd, rgb, depth, utime, self).detach();
        // thread(CameraWrapper::add_framerate_sample, utime, self).detach();
        thread(CameraWrapper::detect_balls, rgb, unordered_cloud, ordered_cloud, utime, 
            self).detach();
    }
}

// Task threads of CameraWrapper implementations

void CameraWrapper::broadcast_rgbd(const Mat rgb, const Mat depth, int64_t utime, 
    CameraWrapper* self)
{
    // Set metadata at start
    if (self->broadcast_rgbd_running_)
    {
        cerr << "broadcast_rgbd took too long!\n";
    }
    self->broadcast_rgbd_running_ = true;

    rgbd_image_t rgbd;

    vision::rgbdToZcmType( rgb, depth, rgbd);

    rgbd.utime = utime;

    // TODO Channel names should be stored in a config file
    // CameraWrapper should get channel name from this config file
    self->zcm_ptr_->publish("TODO_CHANNEL_NAMES", &rgbd);

    self->broadcast_rgbd_running_ = false;
}

void CameraWrapper::add_framerate_sample(int64_t utime, CameraWrapper* self)
{
    CameraWrapper::measure_framerate_.add_sample(utime);
}

MeasureFramerate CameraWrapper::measure_framerate_;

void CameraWrapper::detect_balls(const Mat rgb, const PointCloud<PointXYZ>::Ptr unordered_cloud, 
    const PointCloud<PointXYZ>::Ptr ordered_cloud, int64_t utime, CameraWrapper* self)
{
    // Set metadata at start
    if (self->ball_detection_running_)
    {
        // cerr << "ball detection took too long\n";
        return;
    }
    self->ball_detection_running_ = true;

    // Actual ball detection
    ball_detections_t message = self->ball_detector_.detect(rgb, unordered_cloud, ordered_cloud);
    message.utime = utime;
    /*
    auto profile = [&]()
    {
        int64_t completion_time = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
        cerr << "completion time: " << completion_time - utime << '\n';
        cerr << "num detections: " << message.num_detections << '\n';
    };
    profile();
    */

    self->zcm_ptr_->publish(channel::BALL_DETECTIONS, &message);
    self->ball_detection_running_ = false;
}

// End task threads of CameraWrapper implementations
// End CameraWrapper Section **************************************

// Set up graceful exit code
function<void(int)> sigHandlerImpl;
void sigHandler(int sig) {sigHandlerImpl(sig);}

int main(int argc, char**argv)
{
    string config_file_path = "../config/camera-config.yaml";
    // Parse program options and get config file
    char opt;
    while ((opt = getopt(argc,argv,"abc:d")) != EOF)
    {
        switch(opt)
        {
            case 'c': 
                config_file_path = string(optarg);
                break;
            case 'h': 
                cout << "usuage: ./driver -c <config-file-path (none for default)>"
                    << endl;
                break;
            default:
                cout << "unknown option " << opt << endl;
                break;
        }
    }
    YAML::Node config = YAML::LoadFile(config_file_path);
    // Create zcm instance
    shared_ptr<ZCM> zcm_ptr = shared_ptr<ZCM>(new ZCM("ipc"));
    // Create camera wrappers
    vector<shared_ptr<CameraWrapper>> wrappers;
    wrappers.reserve(4);
    wrappers.emplace_back(new CameraWrapper(config["forward"], config["calibration"], zcm_ptr));
    sleep_for(milliseconds(8));
    wrappers.emplace_back(new CameraWrapper(config["downward"], config["calibration"], zcm_ptr));
    sleep_for(milliseconds(8));
    wrappers.emplace_back(new CameraWrapper(config["other-forward"], config["calibration"], 
        zcm_ptr));
    // Start all the camera wrappers
    for (auto& wrapper : wrappers)
    {
        wrapper->start();
    }
    // Set up sighandler
    atomic_bool running;
    running = true;
    condition_variable cond_var;
    mutex mtx;
    // Relies upon the global sigHandler above
    sigHandlerImpl = [&] (int)
    {
        unique_lock<mutex> lck(mtx);
        running = false;
        cond_var.notify_one();
    };
    // Must set the sigHandlerImpl before setting signal handling
    signal(SIGINT, sigHandler);
    // signal(SIGABRT, sigHandler);
    // signal(SIGSEGV, sigHandler);
    // signal(SIGTERM, sigHandler);
    // Wait until someone tries to terminate the program
    while (running)
    {
        unique_lock<mutex> lck(mtx);
        cond_var.wait(lck);
    }
    // Stop all the camera wrappers
    for (auto& wrapper : wrappers) wrapper->stop();
    // join all the camera wrappers
    for (auto& wrapper : wrappers) wrapper->join();
}