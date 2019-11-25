#pragma once

#include <common/messages/ball_detection_t.hpp>
#include <common/messages/ball_detections_t.hpp>

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class BallDetector
{
public:
    BallDetector();
    BallDetector(float min_radius, float max_radius);
    ball_detections_t detect(const cv::Mat rgb, 
        const pcl::PointCloud<pcl::PointXYZ>::Ptr unordered_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    const float min_radius_;
    const float max_radius_;
};

