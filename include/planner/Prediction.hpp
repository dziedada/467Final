#pragma once
#include <cstdint>
#include <Eigen/Core>

class Prediction
{
public:
    int64_t ball_in_range_time_;
    Eigen::Vector2d ball_inrange_position_;
    Eigen::Vector2d ball_inrange_velocity_;
    Eigen::Vector2d goal_; // Ignore for now
    int64_t utime_; // Time when this was created (not of the detection)
};
