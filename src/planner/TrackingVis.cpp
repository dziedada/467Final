#include <common/colors.hpp>
#include <planner/TrackingVis.hpp>

#include <opencv2/highgui.hpp>

#include <iostream>

using std::cout;
using std::cerr;
using std::endl;

// Coordinate system is the vision coordinate system
// Therefore, image x and y are flipped
constexpr float SCALE = 200;
constexpr int DISPLAY_X = 1000;
constexpr int DISPLAY_Y = 1000;
constexpr int DISPLAY_Y_ZERO = 500;
constexpr int DISPLAY_X_ZERO = 0;
constexpr int X_AXIS = 1;
constexpr int Y_AXIS = 0;
constexpr float PREDICTION_TIME = 0.1; // Units are seconds

const cv::Scalar green(0, 230, 0);
const cv::Scalar orange(200, 100, 20);
const cv::Scalar blue(0, 0, 230);

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
        // Flipped because in vision coordinate system
        int x = ball.coordinate[X_AXIS] * SCALE + DISPLAY_X_ZERO;
        int y = ball.coordinate[Y_AXIS] * SCALE + DISPLAY_Y_ZERO;
        cv::Scalar pixels;
        if (ball.color == color::Green) pixels = green;
        else if (ball.color == color::Orange) pixels = orange;
        else if (ball.color == color::Blue) pixels = blue;
        else
        {
            cerr << "Please update the visualization with the new color\n";
            exit(1);
        }
        // Draw Circle
        cv::circle(display, cv::Point(x, y), 3, pixels, cv::FILLED);
        // Compute Line to position in configured number of seconds
        int future_x = ball.velocity[X_AXIS] * PREDICTION_TIME * SCALE + x;
        int future_y = ball.velocity[Y_AXIS] * PREDICTION_TIME * SCALE + y;
        // Draw the line
        cv::line(display, cv::Point(x, y), cv::Point(future_x, future_y), pixels, 3);
    }
    cv::imshow("Tracking Visualization", display);
}


