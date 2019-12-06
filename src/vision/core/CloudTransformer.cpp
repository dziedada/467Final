#include <vision/core/CloudTransformer.hpp>

#include <iostream>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

using std::cout;
using std::cerr;
using std::endl;
using Eigen::Matrix4f;
using Eigen::Vector4f;
using pcl::PointCloud;
using pcl::PointXYZ;

CloudTransformer::CloudTransformer(const YAML::Node& config)
{
    const YAML::Node& rotation = config["rotation"];
    const YAML::Node& translation = config["translation"];
    for (int y = 0; y < 3; ++y)
    {
        for (int x = 0; x < 3; ++x)
        {
            transform_matrix_(y, x) = rotation[x + y * 3].as<float>();
        }
    }
    for (int y = 0; y < 3; ++y)
    {
        transform_matrix_(y, 3) = translation[y].as<float>();
    }
    for (int x = 0; x < 3; ++x)
    {
        transform_matrix_(3, x) = 0;
    }
    transform_matrix_(3, 3) = 1;
}

PointXYZ CloudTransformer::transform(const PointXYZ& point)
{
    Vector4f vec(point.x, point.y, point.z, 1);
    Vector4f result = transform_matrix_ * vec;
    return PointXYZ(result[0], result[1], result[2]);
}

PointCloud<PointXYZ>::Ptr CloudTransformer::transform(PointCloud<PointXYZ>::Ptr cloud)
{
    PointCloud<PointXYZ>::Ptr output(new PointCloud<PointXYZ>());
    *output = *cloud;
    if (cloud->isOrganized())
    {
        for (size_t y = 0; y < cloud->height; ++y)
        {
            for (size_t x = 0; x < cloud->width; ++x)
            {
                output->at(x, y) = transform(cloud->at(x, y));
            }
        }
    }
    else
    {
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            output->operator[](i) = transform(cloud->operator[](i));
        }
    }
    // pcl::transformPointCloud (*cloud, *output, transform_matrix_);
    return output;
}

PointCloud<PointXYZ> CloudTransformer::transform(const PointCloud<PointXYZ>& cloud)
{
    PointCloud<PointXYZ> output;
    output = cloud;
    if (cloud.isOrganized())
    {
        for (size_t y = 0; y < cloud.height; ++y)
        {
            for (size_t x = 0; x < cloud.width; ++x)
            {
                output.at(x, y) = transform(cloud.at(x, y));
            }
        }
    }
    else
    {
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            output.operator[](i) = transform(cloud.operator[](i));
        }
    }
    // pcl::transformPointCloud (cloud, output, transform_matrix_);
    return output;
}

ball_detection_t CloudTransformer::transform(const ball_detection_t& detection)
{
    Vector4f vec(detection.position[0], detection.position[1], detection.position[2], 1);
    Vector4f result = transform_matrix_ * vec;
    ball_detection_t output = detection;
    for (int i = 0; i < 3; ++i)
    {
        output.position[i] = result[i];
    }
    return output;
}

