#pragma once

#include <common/messages/ball_detection_t.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

class CloudTransformer
{
public:
    CloudTransformer(const YAML::Node& config);
    pcl::PointXYZ transform(const pcl::PointXYZ& point);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ> transform(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    ball_detection_t transform(const ball_detection_t& detection);
private:
    Eigen::Matrix4f transform_matrix_;
};

