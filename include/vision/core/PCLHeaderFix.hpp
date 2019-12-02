#pragma once

#include <pcl/point_types.h>
#include <Eigen/Core>

// This exists because pcl is dank garbage and has an error in the header file
Eigen::Vector3f operator-(const pcl::PointXYZ& l, const pcl::PointXYZ& r)
{
    Eigen::Vector3f result;
    result[0] = l.x - r.x;
    result[1] = l.y - r.y;
    result[2] = l.z - r.z;
    return result;
}

