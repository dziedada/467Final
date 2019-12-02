#include <vision/core/BallDetector.hpp>
#include <common/messages/ball_detection_t.hpp>
#include <common/messages/ball_detections_t.hpp>

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <vector>

class Calibrator
{
public:
    Calibrator(const YAML::Node& config);
    void add_sample(cv::Mat rgb, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr ordered_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr unordered_cloud);
private:
    void compute_extrinsics();
    Eigen::Vector3f green_ball_ground_truth_;
    Eigen::Vector3f orange_ball_ground_truth_;
    int num_samples_needed_;
    int current_num_samples_;
    BallDetector detector_;
    int current_num_iterations_;
    bool computed_extrinsics_ = false;
    std::vector<Eigen::MatrixXf> plane_coefficients_;
    std::vector<ball_detection_t> green_ball_detections_;
    std::vector<ball_detection_t> orange_ball_detections_;
};

