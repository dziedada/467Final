#pragma once

#include <common/messages/ball_detection_t.hpp>
#include <common/messages/ball_detections_t.hpp>

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class BallDetector
{
public:
    ball_detections_t detect(const cv::Mat rgb, 
        const pcl::PointCloud<pcl::PointXYZ>::Ptr unordered_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

