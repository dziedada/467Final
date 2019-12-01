#include <vision/core/utilities.hpp>
#include <vision/core/RealsenseInterface.hpp>
#include <vision/core/BallDetector.hpp>
#include <common/messages/ball_detections_t.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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

using Eigen::MatrixXf;

using vision::CameraPoseData;
using vision::RealsenseInterface;

using namespace std;
using namespace cv;

constexpr double HEIGHT_MIN = 0.009;
constexpr double HEIGHT_MAX = 0.06;

// Set up graceful exit code
function<void(int)> sigHandlerImpl;
void sigHandler(int sig) {sigHandlerImpl(sig);}

Mat maskByHeight(const Mat rgb, PointCloud<PointXYZ>::Ptr ordered_cloud, 
    const MatrixXf& ground_coefs)
{
    if (static_cast<uint32_t>(rgb.cols) != ordered_cloud->width || 
        static_cast<uint32_t>(rgb.rows) != ordered_cloud->height)
    {
        cerr << "Error: ordered_cloud and rgb image must have same dimensions!!!" << endl;
        exit(1);
    }
    Mat mask(rgb.rows, rgb.cols, CV_8UC1);
    // Compute denominator because only depends on plane
    const double plane_x = ground_coefs(0);
    const double plane_y = ground_coefs(1);
    const double plane_z = ground_coefs(2);
    const double plane_d = ground_coefs(3);
    const double denominator = sqrt(plane_y * plane_y + 
                              plane_z * plane_z +
                              plane_d * plane_d);
    for (size_t y = 0; y < ordered_cloud->height; ++y)
    {
        for (size_t x = 0; x < ordered_cloud->width; ++x)
        {
            const PointXYZ& point = ordered_cloud->operator()(x, y);
            double numerator = fabs(plane_x * point.x +
                                    plane_y * point.y +
                                    plane_z * point.z +
                                    plane_d);
            double height = numerator / denominator;
            if (height < HEIGHT_MIN || height > HEIGHT_MAX) mask.at<uchar>(cv::Point(x, y)) = 0;
            else 
            {
                mask.at<uchar>(cv::Point(x, y)) = 1;
                // The below is run if ball color needs to be recalibrated
                // cv::Vec3b pixel = rgb.at<cv::Vec3b>(cv::Point(x, y));
                // cout << static_cast<int>(pixel[0]) << ", " << static_cast<int>(pixel[1]) << ", " 
                //     << static_cast<int>(pixel[2]) << '\n';
            }
        }
    }
    Mat masked(rgb.rows, rgb.cols, rgb.type(), cv::Scalar(0, 0, 0));
    rgb.copyTo(masked, mask);
    return masked;
}

ball_detections_t failure()
{
    ball_detections_t failure;
    failure.num_detections = 0;
    return failure;
}

MatrixXf fitPlane(PointCloud<PointXYZ>::Ptr cloud)
{
    const double inlier_thresh_ = 0.07;
    const double NECESSARY_INLIER_PROPORTION = 0.5;
    // Set up pcl plane segmentation data
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointXYZ> segs;
    // Optional
    segs.setOptimizeCoefficients(true);
    // Mandatory
    segs.setModelType(pcl::SACMODEL_PLANE);
    segs.setMethodType(pcl::SAC_RANSAC);
    segs.setDistanceThreshold(inlier_thresh_);
    segs.setInputCloud(cloud);
    segs.setOptimizeCoefficients(true);
    // Set the cloud to use for plane fitting
    segs.setInputCloud(cloud);
    if (cloud->size())
    {
        // Beginning planar segmentation
        segs.segment(*inliers, *coefficients);
        // Check to see if a model was found
        // If no model found, return failure
        if (inliers->indices.size() < (NECESSARY_INLIER_PROPORTION * cloud->size()))
        {
            return MatrixXf(0, 0);
        }
    }
    else  // If empty cloud was passed in, return failure
    {
        return MatrixXf(0, 0);
    }
    if (coefficients->values.size())
    {
        MatrixXf coefficients_matrix(4, 1);
        for (unsigned i = 0; i < 4; ++i)
        {
            coefficients_matrix(i, 0) = coefficients->values[i];
        }
        return coefficients_matrix;
    }
    return MatrixXf(0, 0);  // No coefficients, return failure
}



namespace
{
    // windows and trackbars name
    const std::string windowName = "Hough Circle Detection Demo";
    const std::string cannyThresholdTrackbarName = "Canny threshold";
    const std::string accumulatorThresholdTrackbarName = "Accumulator Threshold";

    // initial and max values of the parameters of interests.
    const int cannyThresholdInitialValue = 100;
    const int accumulatorThresholdInitialValue = 50;
    const int maxAccumulatorThreshold = 200;
    const int maxCannyThreshold = 255;

    void HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold)
    {
        // will hold the results of the detection
        std::vector<Vec3f> circles;
        // runs the actual detection
        HoughCircles( src_gray, circles, HOUGH_GRADIENT, 1, src_gray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0 );

        // clone the colour, input image for displaying purposes
        Mat display = src_display.clone();
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }

        // shows the results
        imshow( windowName, display);
    }
}

Mat depth_to_grayscale(Mat input)
{
    Mat output(input.rows, input.cols, CV_8UC1);
    short min = numeric_limits<short>::max();
    short max = numeric_limits<short>::min();
    for (size_t y = 0; y < static_cast<size_t>(input.rows); ++y)
    {
        short* ptr = reinterpret_cast<short*>(input.ptr(y));
        for (size_t x = 0; x < static_cast<size_t>(input.cols); ++x)
        {
            min = std::min(min, ptr[x]);
            max = std::max(max, ptr[x]);
        }
    }
    max -= min;
    for (size_t y = 0; y < static_cast<size_t>(input.rows); ++y)
    {
        short* ptr = reinterpret_cast<short*>(input.ptr(y));
        uchar* output_ptr = output.ptr(y);
        for (size_t x = 0; x < static_cast<size_t>(input.cols); ++x)
        {
            float val = static_cast<float>(ptr[x] - min) / max;
            output_ptr[x] = static_cast<uchar>(val * 255);
        }
    }
    return output;
}

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

    //declare and initialize both parameters that are subjects to change
    int cannyThreshold = cannyThresholdInitialValue;
    int accumulatorThreshold = accumulatorThresholdInitialValue;

    // create the main window, and attach the trackbars
    namedWindow( windowName, WINDOW_AUTOSIZE );
    createTrackbar(cannyThresholdTrackbarName, windowName, &cannyThreshold,maxCannyThreshold);
    createTrackbar(accumulatorThresholdTrackbarName, windowName, &accumulatorThreshold, maxAccumulatorThreshold);

    // Wait until someone tries to terminate the program
    while (running)
    {
        // Read frames and spawn off handlers
        // Load the next frame, if no new frame then skip iteration
        if (!camera.loadNext()) continue;
        // Grab everything needed for tasks
        Mat src = camera.getRGB();
        Mat depth = camera.getDepth();
        PointCloud<PointXYZ>::Ptr unordered_cloud = camera.getPointCloudBasic();
        PointCloud<PointXYZ>::Ptr ordered_cloud = camera.getMappedPointCloud();

        MatrixXf coefs = fitPlane(unordered_cloud);
        if (coefs.size() == 0)
        {
            continue;
        }
        // maskByHeight(src, ordered_cloud, coefs);

        // src = depth_to_grayscale(depth);
        // Mat src_gray = src;
        Mat src_gray;

        // Convert it to gray
        cvtColor( src, src_gray, COLOR_BGR2GRAY );
        Mat hsv;
        cvtColor(src, hsv, COLOR_BGR2HSV);
        vector<Mat> channels(4);
        cv::split(hsv, channels);
        channels[3] = src_gray;
        vector<string> names = {"hue", "saturation", "value", "gray"};

        for (size_t i = 0; i < channels.size(); ++i)
        {
            cout << names[i] << '\n';
            // Reduce the noise so we avoid false circle detection
            GaussianBlur(channels[i], channels[i], Size(9, 9), 2, 2 );
            // those parameters cannot be =0
            // so we must check here
            cannyThreshold = std::max(cannyThreshold, 1);
            accumulatorThreshold = std::max(accumulatorThreshold, 1);

            //runs the detection, and update the display
            HoughDetection(channels[i], src, cannyThreshold, accumulatorThreshold);

            waitKey(0);
        }
    }

    return 0;

}

