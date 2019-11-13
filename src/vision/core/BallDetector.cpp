#include <common/messages/ball_detection_t.hpp>
#include <common/messages/ball_detections_t.hpp>
#include <vision/core/BallDetector.hpp>
#include <common/colors.hpp>
#include <vision/core/PCLHeaderFix.hpp>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/geometry.h>

#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <climits>

using pcl::PointCloud;
using pcl::PointXYZ;
using cv::Mat;
using Eigen::MatrixXf;

using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::shared_ptr;
using std::numeric_limits;

constexpr double HEIGHT_MIN = 0.009;
constexpr double HEIGHT_MAX = 0.06;
// const double BALL_RADIUS = 0.02133;
constexpr double BALL_RADIUS_MIN = 0.016;
constexpr double BALL_RADIUS_MAX = 0.04;

const cv::Scalar GREEN_MIN(32, 60, 30);
const cv::Scalar GREEN_MAX(71, 255, 255);
const cv::Scalar ORANGE_MIN(0, 150, 30);
const cv::Scalar ORANGE_MAX(23, 255, 255);

const Mat EROSION_KERNEL = cv::getStructuringElement(cv::MORPH_RECT,
    cv::Size(5, 5));
const Mat DILATION_KERNEL = cv::getStructuringElement(cv::MORPH_RECT,
    cv::Size(7, 7));
const Mat EROSION_KERNEL_2 = cv::getStructuringElement(cv::MORPH_RECT,
    cv::Size(3, 3));

PointXYZ& operator+=(PointXYZ& l, const PointXYZ& r)
{
    l.x += r.x;
    l.y += r.y;
    l.z += r.z;
    return l;
}

PointXYZ operator/(const PointXYZ& l, const float div)
{
    PointXYZ result;
    result.x = l.x / div;
    result.y = l.y / div;
    result.z = l.z / div;
    return result;
}

/* Start Ball Prototype Section **************************************** */

class BallPrototype
{
public:
    BallPrototype(color::Color color_in) : color_ {color_in}, centroid_ {PointXYZ()},
        cloud_ {new PointCloud<PointXYZ>()} {}
    PointXYZ get_raw_centroid();
    float getRawDistToOther(BallPrototype& other);
    float getColoredDistToOther(BallPrototype& other);
    void assimilate(const BallPrototype& other);
    void addElement(int x, int y, const PointXYZ& point);
    int getPixelX();
    int getPixelY();
    const PointCloud<PointXYZ>::Ptr getCloud();
    size_t size() const;
    color::Color color_;
    PointXYZ centroid_;
private:
    PointCloud<PointXYZ>::Ptr cloud_;
    int64_t pixel_x_sum_ = 0;
    int64_t pixel_y_sum_ = 0;
    size_t num_pixels_ = 0;
    bool has_raw_centroid_ = false;
    PointXYZ raw_centroid_;
};

PointXYZ BallPrototype::get_raw_centroid()
{
    if (has_raw_centroid_) return raw_centroid_;
    int num_points = 0;
    PointXYZ accumulator(0, 0, 0);
    for (const auto& point : *cloud_)
    {
        accumulator += point;
        ++num_points;
    }
    raw_centroid_ = accumulator / num_points;
    has_raw_centroid_ = true;
    return raw_centroid_;
}

float BallPrototype::getRawDistToOther(BallPrototype& other)
{
    return pcl::geometry::distance(get_raw_centroid(), other.get_raw_centroid());
}

float BallPrototype::getColoredDistToOther(BallPrototype& other)
{
    if (color_ != other.color_) return numeric_limits<float>::max();
    return pcl::geometry::distance(get_raw_centroid(), other.get_raw_centroid());
}

void BallPrototype::assimilate(const BallPrototype& other)
{
    for (const auto& point : *other.cloud_)
    {
        cloud_->push_back(point);
    }
    pixel_x_sum_ += other.pixel_x_sum_;
    pixel_y_sum_ += other.pixel_y_sum_;
    num_pixels_  += other.num_pixels_;
}

void BallPrototype::addElement(int x, int y, const PointXYZ& point)
{
    ++num_pixels_;
    pixel_x_sum_ += x;
    pixel_y_sum_ += y;
    cloud_->push_back(point);
}

int BallPrototype::getPixelX()
{
    return pixel_x_sum_ / num_pixels_;
}

int BallPrototype::getPixelY()
{
    return pixel_y_sum_ / num_pixels_;
}

const PointCloud<PointXYZ>::Ptr BallPrototype::getCloud()
{
    return cloud_;
}

size_t BallPrototype::size() const
{
    return num_pixels_;
}

// END BALL PROTOTYPE SECTION **********************************************************

BallDetector::BallDetector(bool debug) : 
    min_radius_ {BALL_RADIUS_MIN}, max_radius_ {BALL_RADIUS_MAX}, debug_ {debug} {}
BallDetector::BallDetector() : BallDetector(false) {}
BallDetector::BallDetector(float min_radius, float max_radius, bool debug) 
    : min_radius_ {min_radius}, max_radius_ {max_radius}, debug_ {debug} {}
BallDetector::BallDetector(float min_radius, float max_radius) 
    : BallDetector(min_radius, max_radius, false) {} 

vector<shared_ptr<BallPrototype> > prunePrototypes(
    vector<shared_ptr<BallPrototype> >& prototypes,
    const BallDetector& detector);
vector<shared_ptr<BallPrototype> > binPrototypes(
    const vector<shared_ptr<BallPrototype> >& prototypes,
    const BallDetector& detector);
vector<shared_ptr<BallPrototype> > fitSpheres(const vector<shared_ptr<BallPrototype> >& prototypes,
    const BallDetector& detector);
Mat maskByHeight(const Mat rgb, PointCloud<PointXYZ>::Ptr ordered_cloud, 
    const MatrixXf& ground_coefs);
ball_detections_t failure();

ball_detections_t BallDetector::detect(Mat rgb, PointCloud<PointXYZ>::Ptr unordered_cloud,
    PointCloud<PointXYZ>::Ptr ordered_cloud)
{
    // Plane Fitting to find height of points TODO Disable once extrinsic callibration is complete
    MatrixXf ground_plane = fitPlane(unordered_cloud);
    if (ground_plane.size() == 0)
    {
        if (debug_) cerr << "No ground coefficients...\n";
        return failure();
    }
    // Mask out the pixels corresponding to points outside of the height range
    Mat hsv;
    cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
    Mat masked = maskByHeight(hsv, ordered_cloud, ground_plane);
    // Color Detection
    Mat green_masked;
    Mat orange_masked;
    Mat green_mask;
    Mat orange_mask;
    cv::inRange(masked, GREEN_MIN, GREEN_MAX, green_masked);
    cv::inRange(masked, ORANGE_MIN, ORANGE_MAX, orange_masked);
    cv::threshold(green_masked, green_mask, 0.0, 1.0, cv::THRESH_BINARY);
    cv::threshold(orange_masked, orange_mask, 0.0, 1.0, cv::THRESH_BINARY);
    cv::erode(green_mask, green_mask, EROSION_KERNEL);
    cv::dilate(green_mask, green_mask, DILATION_KERNEL);
    cv::erode(green_mask, green_mask, EROSION_KERNEL_2);
    cv::erode(orange_mask, orange_mask, EROSION_KERNEL);
    cv::dilate(orange_mask, orange_mask, DILATION_KERNEL);
    cv::erode(orange_mask, orange_mask, EROSION_KERNEL_2);
    // Extract Detected masses of color and create BallPrototypes
    uint8_t curr_label = 2;
    vector<shared_ptr<BallPrototype> > prototypes;
    auto getPrototypeIndex = [](size_t index) {return index - 2;};
    auto extractPrototypes = [&](Mat& mask, color::Color ball_color) {
        for (size_t y = 0; y < static_cast<size_t>(mask.rows); ++y)
        {
            uchar* ptr = mask.ptr(y);
            for (size_t x = 0; x < static_cast<size_t>(mask.cols); ++x)
            {
                if (ptr[x] == 1)
                {
                    cv::floodFill(mask, cv::Point(x, y), cv::Scalar(curr_label));
                    prototypes.emplace_back(new BallPrototype(ball_color));
                    prototypes[getPrototypeIndex(curr_label)]->
                        addElement(x, y, ordered_cloud->operator()(x, y));
                    ++curr_label;
                }
                else if (ptr[x] != 0)
                {
                    prototypes[getPrototypeIndex(ptr[x])]->
                        addElement(x, y, ordered_cloud->operator()(x, y));
                }
            }
        }
    };
    extractPrototypes(green_mask, color::Green);
    extractPrototypes(orange_mask, color::Orange);
    // Combine prototypes of the same color that are nearby
    prototypes = binPrototypes(prototypes, *this);
    // Remove prototypes of different colors that are too close to others based on num points
    prototypes = prunePrototypes(prototypes, *this);
    // Ransac Sphere Fitting on point clouds corresponding to detected blobs
    // Prune 'spheres' that aren't likely to be balls // TODO can be merged with the "masking" step
    vector<shared_ptr<BallPrototype> > complete_prototypes = fitSpheres(prototypes, *this);
    // Convert to the returned representation (ball_detections_t)
    ball_detections_t detections;
    for (const auto& prototype : complete_prototypes)
    {
        detections.detections.emplace_back();
        detections.detections.back().color = prototype->color_;
        detections.detections.back().position[0] = prototype->centroid_.x;
        detections.detections.back().position[1] = prototype->centroid_.y;
        detections.detections.back().position[2] = prototype->centroid_.z;
    }
    detections.num_detections = detections.detections.size();
    auto visualize = [&]() 
    {
        cv::namedWindow("Masked", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Green Masked", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Orange Masked", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Green Final", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Orange Final", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Detections", cv::WINDOW_AUTOSIZE);
        Mat green_vis;
        Mat orange_vis;
        cv::threshold(green_mask, green_vis, 0.0, 254.0, cv::THRESH_BINARY);
        cv::threshold(orange_mask, orange_vis, 0.0, 254.0, cv::THRESH_BINARY);
        Mat detection_image = rgb.clone();
        for (auto& prototype : complete_prototypes)
        {
            cv::circle(detection_image, cv::Point(prototype->getPixelX(), prototype->getPixelY()),
                15, cv::Scalar(50, 100, 250), cv::FILLED);
        }
        imshow("Masked", masked);
        imshow("Green Masked", green_masked);
        imshow("Orange Masked", orange_masked);
        imshow("Green Final", green_vis);
        imshow("Orange Final", orange_vis);
        imshow("Detections", detection_image);
        cv::waitKey(0);
    };
    if (debug_) visualize();
    return detections;
}

vector<shared_ptr<BallPrototype> > prunePrototypes(
    vector<shared_ptr<BallPrototype> >& prototypes,
    const BallDetector& detector)
{
    // Sort prototypes based on cloud size so that biggest are in front
    // That way nothing needs to be removed from the processed vector
    std::sort(prototypes.begin(), prototypes.end(), 
        [](const shared_ptr<BallPrototype>& l, const shared_ptr<BallPrototype>& r)
        {
            return l->size() > r->size();
        });
    // Perform the selective keeping which performs the pruning
    vector<shared_ptr<BallPrototype> > processed_prototypes;
    for (const auto& prototype : prototypes)
    {
        bool is_good = true;
        for (auto& other : processed_prototypes)
        {
            if (prototype->getRawDistToOther(*other) < detector.max_radius_)
            {
                is_good = false;
                break;
            }
        }
        if (is_good) processed_prototypes.push_back(prototype);
    }
    return processed_prototypes;
}

vector<shared_ptr<BallPrototype> > binPrototypes(
    const vector<shared_ptr<BallPrototype> >& prototypes,
    const BallDetector& detector)
{
    vector<shared_ptr<BallPrototype> > processed_prototypes;
    for (const auto& prototype : prototypes)
    {
        float best_dist = 99999.0;
        shared_ptr<BallPrototype> best_prototype;
        for (auto& candidate : processed_prototypes)
        {
            float dist = candidate->getColoredDistToOther(*prototype);
            if (dist < best_dist)
            {
                best_dist = dist;
                best_prototype = candidate;
            }
        }
        if (best_dist <= detector.max_radius_)
        {
            best_prototype->assimilate(*prototype);
        }
        else
        {
            processed_prototypes.push_back(prototype);
        }
    }
    return processed_prototypes;
}

vector<shared_ptr<BallPrototype> > fitSpheres(const vector<shared_ptr<BallPrototype> >& prototypes,
    const BallDetector& detector)
{
    vector<shared_ptr<BallPrototype> > complete_prototypes_;
    auto fitSphere = [&](const shared_ptr<BallPrototype>& prototype)
    {
        if (!prototype->size()) {cerr << "empty cloud...\n"; return;}
        const double inlier_thresh_ = 0.04;
        // const double NECESSARY_INLIER_PROPORTION = 0.1;
        // Set up pcl plane segmentation data
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<PointXYZ> segs;
        // Optional
        segs.setOptimizeCoefficients(true);
        // Mandatory
        segs.setModelType(pcl::SACMODEL_SPHERE);
        segs.setMethodType(pcl::SAC_RANSAC);
        segs.setDistanceThreshold(inlier_thresh_);
        segs.setMaxIterations(1000);
        segs.setRadiusLimits(detector.min_radius_, detector.max_radius_);
        // Old distanceThreshold at 0.3
        segs.setOptimizeCoefficients(true);
        // Set the cloud to use for plane fitting
        segs.setInputCloud(prototype->getCloud());
        // Beginning planar segmentation
        segs.segment(*inliers, *coefficients);
        // Check to see if a model was found
        // If no model found, return failure
        /*
        // The issue with this is that even the real detections have very low inlier proportion...
        if (inliers->indices.size() < (NECESSARY_INLIER_PROPORTION * prototype->size()))
        {
            if (detector.debug_) cerr << "Not enough inliers" << '\n'; // TODO DEBUG
            return;
        }
        */
        if (coefficients->values.size())
        {
            complete_prototypes_.push_back(prototype);
            complete_prototypes_.back()->centroid_.x = coefficients->values[0];
            complete_prototypes_.back()->centroid_.y = coefficients->values[1];
            complete_prototypes_.back()->centroid_.z = coefficients->values[2];
        }
        else
        {
            if (detector.debug_) cerr << "No coefficients" << '\n';
        }
        
    };
    // This loop populates the complete_prototypes_ vector
    for (const auto& prototype: prototypes)
    {
        fitSphere(prototype);
    }
    return complete_prototypes_;
}

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

MatrixXf BallDetector::fitPlane(PointCloud<PointXYZ>::Ptr cloud)
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

