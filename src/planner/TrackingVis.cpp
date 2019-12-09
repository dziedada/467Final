#include <common/colors.hpp>
#include <planner/TrackingVis.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <chrono>

using std::cout;
using std::cerr;
using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::seconds;

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
constexpr int64_t MAX_PRED_TIME_DIFF = 400; // Units are milliseconds

const cv::Scalar green(0, 230, 0);
const cv::Scalar orange(20, 100, 200);
const cv::Scalar blue(230, 0, 0);
const cv::Scalar white(230, 230, 230);
const cv::Scalar detection_color = white;
const cv::Scalar prediction_color(230, 30, 230);

bool shouldPlotPrediction(const Prediction& pred);

TrackingVis::TrackingVis(const std::shared_ptr<std::condition_variable>& cond_var,
    const std::shared_ptr<std::mutex>& mtx, const std::vector<Ball>& balls) :
    cond_var_ {cond_var}, mtx_ {mtx}, balls_ {balls}
{
    cv::namedWindow("Tracking Visualization", cv::WINDOW_AUTOSIZE);
}

void TrackingVis::update()
{
    cv::Mat display(DISPLAY_X, DISPLAY_Y, CV_8UC3, cv::Scalar(0, 0, 0));
    // Draw Grid Lines
    cv::line(display, cv::Point(DISPLAY_X_ZERO, 0), cv::Point(DISPLAY_X_ZERO, DISPLAY_Y), 
        white, LINE_THICKNESS);
    cv::line(display, cv::Point(0, DISPLAY_Y_ZERO), cv::Point(DISPLAY_X, DISPLAY_Y_ZERO), 
        white, LINE_THICKNESS);
    for (const Ball& ball : balls_)
    {
        // Compute ball location
        int x = ball.coordinate[X_AXIS] * SCALE + DISPLAY_X_ZERO;
        int y = ball.coordinate[Y_AXIS] * SCALE + DISPLAY_Y_ZERO;
        // Compute detection / measurement location
        int detection_x = ball.meas[X_AXIS] * SCALE + DISPLAY_X_ZERO;
        int detection_y = ball.meas[Y_AXIS] * SCALE + DISPLAY_Y_ZERO;
        // Compute prediction location
        int prediction_x = ball.reachPrediction.ball_inrange_position_[X_AXIS] * SCALE + 
            DISPLAY_X_ZERO;
        int prediction_y = ball.reachPrediction.ball_inrange_position_[Y_AXIS] * SCALE +
            DISPLAY_Y_ZERO;
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
        // Draw Circle for prediction and line connecting to position
        if (shouldPlotPrediction(ball.reachPrediction))
        {
            cv::circle(display, cv::Point(prediction_x, prediction_y), CIRCLE_RADIUS, 
                prediction_color);
            cv::line(display, cv::Point(prediction_x, prediction_y), cv::Point(x, y), 
                prediction_color, LINE_THICKNESS);
        }
        // Draw Circle for position
        cv::circle(display, cv::Point(x, y), CIRCLE_RADIUS, pixels, cv::FILLED);
        // Compute Line to position in configured number of seconds
        int future_x = ball.velocity_coord_avg[X_AXIS] * PREDICTION_TIME * SCALE + x;
        int future_y = ball.velocity_coord_avg[Y_AXIS] * PREDICTION_TIME * SCALE + y;
        // Draw the line to the prediction
        cv::line(display, cv::Point(x, y), cv::Point(future_x, future_y), pixels, LINE_THICKNESS);
    }
    cv::imshow("Tracking Visualization", display);
    cv::waitKey(1);
}

bool shouldPlotPrediction(const Prediction& pred)
{
    milliseconds curr_time = 
        duration_cast<milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    milliseconds pred_time = duration_cast<milliseconds>(microseconds(pred.ball_in_range_time_));
    return ((pred_time - curr_time).count() <= MAX_PRED_TIME_DIFF);
}

