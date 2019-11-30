#include <vision/core/Calibrator.hpp>
#include <vision/core/BallDetector.hpp>
#include <common/messages/ball_detection_t.hpp>
#include <common/messages/ball_detections_t.hpp>
#include <common/colors.hpp>

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <vector>
#include <memory>
#include <climits>
#include <algorithm>

using cv::Mat;
using pcl::PointCloud;
using pcl::PointXYZ;
using Eigen::MatrixXf;
using Eigen::Matrix3f;
using Eigen::Vector3f;

using std::cout;
using std::endl;
using std::cerr;
using std::vector;
using std::shared_ptr;
using std::numeric_limits;

constexpr double PI = 3.14159265359;

Calibrator::Calibrator(const YAML::Node& config) :
    green_ball_ground_truth_ {Vector3f(
        config["green_ground_truth"]["x"].as<float>(),
        config["green_ground_truth"]["y"].as<float>(),
        config["green_ground_truth"]["z"].as<float>()
        )},
    orange_ball_ground_truth_ {Vector3f(
        config["orange_ground_truth"]["x"].as<float>(),
        config["orange_ground_truth"]["y"].as<float>(),
        config["orange_ground_truth"]["z"].as<float>()
    )},
    num_samples_needed_ {config["num_samples_required"].as<int>()},
    current_num_samples_ {0},
    detector_ {BallDetector(config["min_ball_radius"].as<float>(),
                            config["max_ball_radius"].as<float>())},
    current_num_iterations_ {0}
{
    plane_coefficients_.reserve(static_cast<size_t>(num_samples_needed_));
    green_ball_detections_.reserve(static_cast<size_t>(num_samples_needed_));
    orange_ball_detections_.reserve(static_cast<size_t>(num_samples_needed_));
}


void Calibrator::add_sample(cv::Mat rgb, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr ordered_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr unordered_cloud)
{
    if (computed_extrinsics_) return;
    ++current_num_iterations_;
    ball_detections_t detections = detector_.detect(rgb, unordered_cloud, ordered_cloud);
    for (const ball_detection_t& detection : detections.detections)
    {
        if (detection.color == color::Green) green_ball_detections_.push_back(detection);
        else if (detection.color == color::Orange) orange_ball_detections_.push_back(detection);
    }
    MatrixXf coefs = detector_.fitPlane(unordered_cloud);
    if (coefs.size() != 0) plane_coefficients_.push_back(coefs);
    current_num_samples_ = std::min(plane_coefficients_.size(), 
        std::min(green_ball_detections_.size(), orange_ball_detections_.size()));
    if (current_num_samples_ == num_samples_needed_) compute_extrinsics();
    if (current_num_iterations_ > 3 * num_samples_needed_) 
        cerr << "at > 3 * num_samples_needed iterations, something may be wrong...\n";
}

class Bin
{
public:
    Bin(float x, float y, float z, color::Color ball_color)
    {
        centroid_.position[0] = x;
        centroid_.position[1] = y;
        centroid_.position[2] = z;
        centroid_.color = ball_color;
    }
    const ball_detection_t& getCentroid() const {return centroid_;}
    color::Color getColor() const {return static_cast<color::Color>(centroid_.color);}
    size_t size() const {return detections_.size();}
    double distToOther(const ball_detection_t& other) const
    {
        double x_diff = other.position[0] - centroid_.position[0];
        double y_diff = other.position[1] - centroid_.position[1];
        double z_diff = other.position[2] - centroid_.position[2];
        return sqrt((x_diff * x_diff) + (y_diff * y_diff) + (z_diff * z_diff));
    }
    void bin(const ball_detection_t& detection) {detections_.push_back(detection);}
    void computeCentroid()
    {
        if (size() == 0) return;
        double x = 0;
        double y = 0;
        double z = 0;
        for (const auto& detection : detections_)
        {
            x += detection.position[0];
            y += detection.position[1];
            z += detection.position[2];
        }
        x /= detections_.size();
        y /= detections_.size();
        z /= detections_.size();
        centroid_.position[0] = x;
        centroid_.position[1] = y;
        centroid_.position[2] = z;
    }
    void clear() {detections_.clear();}
    void reinitialize() {computeCentroid(); clear();}
private:
    ball_detection_t centroid_;
    vector<ball_detection_t> detections_;
};

void Calibrator::compute_extrinsics()
{
    computed_extrinsics_ = true;
    if (green_ball_detections_.size() == 0 ||
        orange_ball_detections_.size() == 0 ||
        plane_coefficients_.size() == 0)
    {
        cout << "Calibration failed..." << '\n';
        cout << "green ball detections: " << green_ball_detections_.size() << '\n';
        cout << "orange ball detections: " << orange_ball_detections_.size() << '\n';
        cout << "plane coefficients: " << plane_coefficients_.size() << '\n';
        return;
    }
    // Average the ball detections (to reduce the effect of camera depth noise)
    // Bin Centroids and then only keep the one with the highest number of detections
    auto bin_size_greater = [](const shared_ptr<Bin> &l, const shared_ptr<Bin>& r)
    {
        return l->size() > r->size();
    };
    auto herd = [](vector<shared_ptr<Bin> >& bins, const ball_detection_t& detection, double thresh)
    {
        double best_dist = numeric_limits<double>::max();
        shared_ptr<Bin> best_bin;
        for (const auto& bin : bins)
        {
            double dist = bin->distToOther(detection);
            if (dist < best_dist)
            {
                best_dist = dist;
                best_bin = bin;
            }
        }
        if (best_dist < thresh)
        {
            best_bin->bin(detection);
        }
        else
        {
            bins.emplace_back(new Bin(detection.position[0], detection.position[1], 
                detection.position[2], static_cast<color::Color>(detection.color)));
            bins.back()->bin(detection);
        }
    };
    auto prune_bins = [bin_size_greater](vector<shared_ptr<Bin> >& bins, double thresh)
    {
        std::sort(bins.begin(), bins.end(), bin_size_greater);
        vector<shared_ptr<Bin> > pruned_bins;
        for (const auto& bin : bins)
        {
            bool is_unique_or_largest = true;
            for (const auto& other_bin : pruned_bins)
            {
                if (bin->distToOther(other_bin->getCentroid()) < thresh)
                {
                    is_unique_or_largest = false;
                    break;
                }
            }
            if (is_unique_or_largest) pruned_bins.push_back(bin);
        }
        return pruned_bins;
    };
    auto remove_empty_bins = [](const vector<shared_ptr<Bin> >& bins)
    {
        vector<shared_ptr<Bin> > pruned_bins;
        for (const auto& bin : bins)
        {
            if (bin->size() > 0) pruned_bins.push_back(bin);
        }
        return pruned_bins;
    };
    auto filter_detections = [&](const vector<ball_detection_t> detections, double thresh)
    {
        vector<shared_ptr<Bin> > bins;
        for (int i = 0; i < 10; ++i)
        {
            for (auto& bin : bins) bin->clear();
            for (const auto& detection : detections) herd(bins, detection, thresh);
            bins = remove_empty_bins(bins);
            for (auto& bin : bins) bin->computeCentroid();
            bins = prune_bins(bins, thresh);
        }
        std::sort(bins.begin(), bins.end(), bin_size_greater);
        return bins.front()->getCentroid();
    };
    const double _ball_radius = this->detector_.max_radius_;
    ball_detection_t green_centroid = filter_detections(green_ball_detections_, _ball_radius);
    ball_detection_t orange_centroid = filter_detections(orange_ball_detections_, _ball_radius);
    // Compute the height from the plane coefficients
    // Compute the roll and pitch from the plane coefficients
    vector<float> heights; heights.reserve(num_samples_needed_);
    vector<float> rolls; rolls.reserve(num_samples_needed_);
    vector<float> pitches; pitches.reserve(num_samples_needed_);
    auto extract_plane_data = [&](const MatrixXf& coefs)
    {
        if (coefs(2) == 0.f)
        {
            return;
        }
        float height = abs(coefs(2));
        // assuming plane normal of camera is [0,0,1]
        float xc = 0, yc = 0, zc = 1; // Camera
        // Ground
        float xg = coefs(0);
        float yg = coefs(1);
        float zg = coefs(2);
        // Computation
        float roll = acos(((xg * xc) + (zg * zc)) / sqrt((xg * xg) + (zg * zg)));
        float pitch = acos(((yg * yc) + (zg * zc)) / sqrt((yg * yg) + (zg * zg)));
        if (yg < 0) pitch *= -1;
        if (xg < 0) roll *= -1;
        heights.push_back(height);
        rolls.push_back(roll);
        pitches.push_back(pitch);
    };
    for (auto& coefs : plane_coefficients_) extract_plane_data(coefs);
    auto filter_1d_data = [&](const vector<float>& data, double thresh)
    {
        // Convert to ball_detection_t to re-use the filtering code above
        vector<ball_detection_t> detections;
        for (float datum : data)
        {
            ball_detection_t detection;
            detection.position[0] = datum;
            detection.position[1] = 0;
            detection.position[2] = 0;
            detection.color = 0;
            detections.push_back(detection);
        }
        // Use the filtering for 2d data
        ball_detection_t centroid = filter_detections(detections, thresh);
        return centroid.position[0];
    };
    // Average the height, roll, and pitch computations rejecting outliers
    float filtered_height = filter_1d_data(heights, 0.02);
    float filtered_roll = filter_1d_data(rolls, 0.1);
    float filtered_pitch = filter_1d_data(pitches, 0.1);
    // Create Rotation matrices with pitch and roll and adjust filtered ball detections
    Matrix3f x_rot; // pitch fix
    Matrix3f z_rot; // roll fix
    Matrix3f coord_change; // Changes to world coordinate frame without extrinsics
    Matrix3f yaw_rot; // yaw fix
    // Set pitch fix matrix
    float pitch_fix = (-PI / 2) - filtered_pitch;
    float x_angle = -1 * pitch_fix;
    x_rot(0, 0) = 1; x_rot(0, 1) = 0; x_rot(0, 2) = 0;
    x_rot(1, 0) = 0; x_rot(1, 1) = cos(x_angle); x_rot(1, 2) = -sin(x_angle);
    x_rot(2, 0) = 0; x_rot(2, 1) = sin(x_angle); x_rot(2, 2) = cos(x_angle);
    // Set roll fix matrix
    float roll_fix = -filtered_roll;
    float z_angle = -1 * roll_fix;
    z_rot(0, 0) = cos(z_angle); z_rot(0, 1) = -sin(z_angle); z_rot(0, 2) = 0;
    z_rot(1, 0) = sin(z_angle); z_rot(1, 1) = cos(z_angle); z_rot(1, 2) = 0;
    z_rot(2, 0) = 0; z_rot(2, 1) = 0; z_rot(2, 2) = 1;
    // Set coord_change matrix
    coord_change(0, 0) = 0; coord_change(0, 1) = 0; coord_change(0, 2) = 1;
    coord_change(1, 0) = 1; coord_change(1, 1) = 0; coord_change(1, 2) = 0;
    coord_change(2, 0) = 0; coord_change(2, 1) = 1; coord_change(2, 2) = 0;
    // Adjust the detected positions of the balls
    auto linear_transform_ball = [](const Matrix3f& tran, const ball_detection_t& detection)
    {
        
        Vector3f result(tran * 
            Vector3f(detection.position[0], detection.position[1], detection.position[2]));
        ball_detection_t transformed_detection;
        for (size_t i = 0; i < 3; ++i) transformed_detection.position[i] = detection.position[i];
        transformed_detection.color = detection.color;
        transformed_detection.utime = detection.utime;
        return transformed_detection;
    };
    Matrix3f x_z_and_coord = coord_change * x_rot * z_rot;
    green_centroid = linear_transform_ball(x_z_and_coord, green_centroid);
    orange_centroid = linear_transform_ball(x_z_and_coord, orange_centroid);
    // Compute the yaw using the angle of the line connecting the balls
    Vector3f connector = 
        Vector3f(green_centroid.position[0], green_centroid.position[1], 
            green_ball_ground_truth_[2]) - 
        Vector3f(orange_centroid.position[0], orange_centroid.position[1],
            orange_ball_ground_truth_[2]);
    float connector_angle = atan2(connector[0], connector[1]);
    float yaw_angle = -1 * connector_angle;
    yaw_rot(0, 0) = cos(yaw_angle); yaw_rot(0, 1) = -sin(yaw_angle); yaw_rot(0, 2) = 0;
    yaw_rot(1, 0) = sin(yaw_angle); yaw_rot(1, 1) = cos(yaw_angle); yaw_rot(1, 2) = 0;
    yaw_rot(2, 0) = 0; yaw_rot(2, 1) = 0; yaw_rot(2, 2) = 1;
    // Apply the yaw fix
    green_centroid = linear_transform_ball(yaw_rot, green_centroid);
    orange_centroid = linear_transform_ball(yaw_rot, orange_centroid);
    // Compute the horizontal translations using the rotation adjusted ball positions
    float x_diff = (
        (green_ball_ground_truth_[0] - green_centroid.position[0]) +
        (orange_ball_ground_truth_[0] - orange_centroid.position[0])
        ) / 2;
    float y_diff = (
        (green_ball_ground_truth_[1] - green_centroid.position[1]) +
        (orange_ball_ground_truth_[1] - orange_centroid.position[1])
        ) / 2;
    // Create the rotation matrix
    Matrix3f rotation_matrix = yaw_rot * coord_change * x_rot * z_rot;
    // Create the translation vector
    Vector3f translation(x_diff, y_diff, filtered_height);
    // Test the created rotation and translation on the balls and plane inliers
    // TODO
    // Print out the results of the computations
    cout << "rotation matrix: "<< rotation_matrix << ", translation: " << translation << '\n';
}

