#include <common/colors.hpp>
#include <planner/TrackingVis.hpp>

#include <opencv2/highgui.hpp>

#include <iostream>

using std::cout;
using std::cerr;
using std::endl;

// Coordinate system is the vision coordinate system
// Therefore, image x and y are flipped
constexpr float SCALE = 100;
constexpr int DISPLAY_X = 500;
constexpr int DISPLAY_Y = 500;
constexpr int DISPLAY_Y_ZERO = 250;
constexpr int DISPLAY_X_ZERO = 250;
constexpr int X_AXIS = 1;
constexpr int Y_AXIS = 0;
constexpr float PREDICTION_TIME = 0.1; // Units are seconds
constexpr int LINE_THICKNESS = 1;
constexpr int CIRCLE_RADIUS = 3;

const cv::Scalar green(0, 230, 0);
const cv::Scalar orange(200, 100, 20);
const cv::Scalar blue(0, 0, 230);
const cv::Scalar white(230, 230, 230);
const cv::Scalar detection_color = white;

TrackingVis::TrackingVis(const std::shared_ptr<std::condition_variable>& cond_var,
    const std::shared_ptr<std::mutex>& mtx, const std::vector<Ball>& balls) :
    cond_var_ {cond_var}, mtx_ {mtx}, balls_ {balls}
{
    cv::namedWindow("Tracking Visualization", cv::WINDOW_AUTOSIZE);
}

void TrackingVis::update()
{
    cv::Mat display(DISPLAY_X, DISPLAY_Y, CV_8UC3, cv::Scalar(0, 0, 0));
    for (const Ball& ball : balls_)
    {
        // Compute ball location
        int x = ball.coordinate[X_AXIS] * SCALE + DISPLAY_X_ZERO;
        int y = ball.coordinate[Y_AXIS] * SCALE + DISPLAY_Y_ZERO;
        // Compute detection / measurement location
        int detection_x = ball.meas.at<double>(X_AXIS) * SCALE + DISPLAY_X_ZERO;
        int detection_y = ball.meas.at<double>(Y_AXIS) * SCALE + DISPLAY_Y_ZERO;
        // Select pixel color based on ball color
        cv::Scalar pixels;
        if (ball.color == color::Green) pixels = green;
        else if (ball.color == color::Orange) pixels = orange;
        else if (ball.color == color::Blue) pixels = blue;
        else
        {
            cerr << "Please update the visualization with the new color\n";
            exit(1);
        }
        // Draw Circle for detection
        cv::circle(display, cv::Point(detection_x, detection_y), CIRCLE_RADIUS, pixels);
        // Draw line connecting detection to filtered position
        // cv::line(display, cv::Point(x, y), cv::Point(detection_x, detection_y), detection_color, 
        //    LINE_THICKNESS);
        // Draw Circle for position
        cv::circle(display, cv::Point(x, y), CIRCLE_RADIUS, pixels, cv::FILLED);
        // Compute Line to position in configured number of seconds
        int future_x = ball.velocity[X_AXIS] * PREDICTION_TIME * SCALE + x;
        int future_y = ball.velocity[Y_AXIS] * PREDICTION_TIME * SCALE + y;
        // Draw the line to the prediction
        cv::line(display, cv::Point(x, y), cv::Point(future_x, future_y), pixels, LINE_THICKNESS);
    }
    cv::imshow("Tracking Visualization", display);
    cv::waitKey(1);
}


