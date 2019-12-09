#pragma once

#include <planner/Prediction.hpp>

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Core>

enum state {idle, prepare, punch};

class OuterLoopController
{
public:
    OuterLoopController(lcm::LCM* lcm_ptr_in);

    // (x, y, wrist_angle)
    state curr_state_;
    Eigen::Vector3d cur_pose_;
    lcm::LCM* lcm_ptr_;
    Prediction pred_;
    int64_t prev_pred_time_;
    int mode_;

    void runStateMachine();
private:
    void update_target(const Prediction& new_target);

    void update_curpose(const Eigen::Vector3d& new_pose);

    void run_idle();

    // bool diff(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

    Eigen::Vector3d calc_prep_pose(const Prediction& pred);

    Eigen::Vector3d calc_punch_pose(const Prediction& pred);

    void run_prepare();

    void run_punch();

    void publishPlan(const Eigen::Vector3d& arm_pose);
};