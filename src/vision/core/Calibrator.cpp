#include <vision/core/Calibrator.hpp>
#include <vision/core/BallDetector.hpp>
#include <common/messages/ball_detection_t.hpp>
#include <common/messages/ball_detections_t.hpp>
#include <common/colors.hpp>

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <memory>
#include <climits>
#include <algorithm>

using cv::Mat;
using pcl::PointCloud;
using pcl::PointXYZ;
using Eigen::MatrixXf;

using std::vector;
using std::shared_ptr;
using std::numeric_limits;

Calibrator::Calibrator(const YAML::Node& config) :
    green_ball_ground_truth_ {PointXYZ(
        config["green_ground_truth"]["x"].as<float>(),
        config["green_ground_truth"]["y"].as<float>(),
        config["green_ground_truth"]["z"].as<float>()
        )},
    orange_ball_ground_truth_ {PointXYZ(
        config["orange_ground_truth"]["x"].as<float>(),
        config["orange_ground_truth"]["y"].as<float>(),
        config["orange_ground_truth"]["z"].as<float>()
    )},
    num_samples_needed_ {config["num_samples_required"].as<int>()},
    current_num_samples_ {0},
    detector_ {BallDetector(config["min_ball_radius"].as<float>(),
                            config["max_ball_radius"].as<float>())}
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
    ++current_num_samples_;
    ball_detections_t detections = detector_.detect(rgb, unordered_cloud, ordered_cloud);
    for (const ball_detection_t& detection : detections.detections)
    {
        if (detection.color == color::Green) green_ball_detections_.push_back(detection);
        else if (detection.color == color::Orange) orange_ball_detections_.push_back(detection);
    }
    MatrixXf coefs = detector_.fitPlane(unordered_cloud);
    if (coefs.size() != 0) plane_coefficients_.push_back(coefs);
    if (current_num_samples_ == num_samples_needed_) compute_extrinsics();
}

class Bin
{
public:
    Bin(float x, float y, color::Color ball_color);
    const ball_detection_t& getCentroid() const {return centroid_;}
    color::Color getColor() const {return static_cast<color::Color>(centroid_.color);}
    size_t size() const {return detections_.size();}
    double distToOther(const ball_detection_t& other) const
    {
        double x_diff = other.position[0] - centroid_.position[0];
        double y_diff = other.position[1] - centroid_.position[1];
        return sqrt(x_diff * x_diff + y_diff * y_diff);
    }
    void bin(const ball_detection_t& detection) {detections_.push_back(detection);}
    void computeCentroid()
    {
        if (size() == 0) return;
        double x = 0;
        double y = 0;
        for (const auto& detection : detections_)
        {
            x += detection.position[0];
            y += detection.position[1];
        }
        x /= detections_.size();
        y /= detections_.size();
        centroid_.position[0] = x;
        centroid_.position[1] = y;
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
    // Average the ball detections (to reduce the effect of camera depth noise)
    // Bin Centroids and then only keep the one with the highest number of detections
    auto bin_size_greater = [](const shared_ptr<Bin> &l, const shared_ptr<Bin>& r)
    {
        return l->size() > r->size();
    };
    auto bin = [this](vector<shared_ptr<Bin> >& bins, const ball_detection_t& detection)
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
        if (best_dist < this->detector_.max_radius_)
        {
            best_bin->bin(detection);
        }
        else
        {
            bins.emplace_back(detection.position[0], detection.position[1], detection.color);
            bins.back()->bin(detection);
        }
    };
    auto prune_bins = [this, bin_size_greater](vector<shared_ptr<Bin> >& bins)
    {
        std::sort(bins.begin(), bins.end(), bin_size_greater);
        vector<shared_ptr<Bin> > pruned_bins;
        for (const auto& bin : bins)
        {
            bool is_unique_or_largest = true;
            for (const auto& other_bin : pruned_bins)
            {
                if (bin->distToOther(other_bin->getCentroid()) < this->detector_.max_radius_)
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
    auto filter_detections = [&](const vector<ball_detection_t> detections)
    {
        vector<shared_ptr<Bin> > bins;
        for (int i = 0; i < 6; ++i)
        {
            for (auto& bin : bins) bin->clear();
            for (const auto& detection : detections) bin(bins, detection);
            bins = remove_empty_bins(bins);
            for (auto& bin : bins) bin->computeCentroid();
            bins = prune_bins(bins);
        }
        std::sort(bins.begin(), bins.end(), bin_size_greater);
        return bins.front()->getCentroid();
    };
    ball_detection_t green_centroid = filter_detections(green_ball_detections_);
    ball_detection_t orange_centroid = filter_detections(orange_ball_detections_);
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
    // Average the height, roll, and pitch computations rejecting outliers
    // Compute the yaw using the angle of the line connecting the balls
    // Compute the horizontal translation using the rotation adjusted ball positions
    // Create the rotation matrix
    // Create the translation vector
    // Test the created rotation and translation on the balls and plane inliers
    // Print out the results of the computations
}
