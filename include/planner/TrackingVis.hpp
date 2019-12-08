#pragma once

#include <planner/ball.hpp>

#include <Eigen/Core>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <vector>

class TrackingVis
{
public:
    TrackingVis(const std::shared_ptr<std::condition_variable>& cond_var,
        const std::shared_ptr<std::mutex>& mtx, const std::vector<Ball>& balls);
    void update();
    std::shared_ptr<std::condition_variable> cond_var_;
    std::shared_ptr<std::mutex> mtx_;
private:
    const std::vector<Ball>& balls_;
};