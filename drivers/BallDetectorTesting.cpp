#include <vision/core/utilities.hpp>
#include <vision/core/RealsenseInterface.hpp>
#include <vision/core/BallDetector.hpp>
#include <common/messages/ball_detections_t.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
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

using cv::Mat;

using pcl::PointCloud;
using pcl::PointXYZ;

using vision::CameraPoseData;
using vision::RealsenseInterface;

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
    RealsenseInterface camera(config["forward"]);
    BallDetector ball_detector(true);
    camera.disableAutoExposure();
    // Set up sighandler
    atomic_bool running;
    running = true;
    // Relies upon the global sigHandler above
    sigHandlerImpl = [&] (int)
    {
        running = false;
    };
    // Must set the sigHandlerImpl before setting signal handling
    signal(SIGINT, sigHandler);
    // Wait until someone tries to terminate the program
    while (running)
    {
        // Read frames and spawn off handlers
        // Load the next frame, if no new frame then skip iteration
        if (!camera.loadNext()) continue;
        // Grab everything needed for tasks
        Mat rgb = camera.getRGB();
        Mat depth = camera.getDepth();
        PointCloud<PointXYZ>::Ptr unordered_cloud = camera.getPointCloudBasic();
        PointCloud<PointXYZ>::Ptr ordered_cloud = camera.getMappedPointCloud();
        int64_t utime = camera.getUTime();

        ball_detections_t message = ball_detector.detect(rgb, unordered_cloud, ordered_cloud);
        auto profile = [&]()
        {
            int64_t completion_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
            cerr << "completion time: " << completion_time - utime << '\n';
            cerr << "num detections: " << message.num_detections << '\n';
        };
        profile();
    }
}


