#pragma once

#include <common/messages/ball_detection_t.hpp>
#include <common/messages/ball_detections_t.hpp>

#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

class BallDetector
{
public:
    BallDetector();
    BallDetector(bool debug);
    BallDetector(float min_radius, float max_radius);
    BallDetector(float min_radius, float max_radius, bool debug);
    BallDetector(bool debug, const YAML::Node& config);
    ball_detections_t detect(const cv::Mat rgb, 
        const pcl::PointCloud<pcl::PointXYZ>::Ptr unordered_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    static Eigen::MatrixXf fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    const bool debug_;
    const float min_radius_;
    const float max_radius_;
    const bool plane_mask_;
    const float x_min_;
    const float x_max_;
    const float y_min_;
    const float y_max_;
    const float z_min_;
    const float z_max_;
};

