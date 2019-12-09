#include <planner/OuterLoopController.hpp>
#include <common/messages/arm_path_t.hpp>
#include <common/message_channels.hpp>

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <limits>

using Eigen::Vector2d;
using Eigen::Vector3d;

using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::duration_cast;
using std::numeric_limits;

OuterLoopController::OuterLoopController(lcm::LCM* lcm_ptr_in) : curr_state_ {state::idle},
    cur_pose_ {Vector3d(-1, -1, 0)}, lcm_ptr_ {lcm_ptr_in}
{
    pred_.ball_in_range_time_ = numeric_limits<int64_t>::max();
}

void OuterLoopController::update_curpose(const Vector3d& new_pose) {
    cur_pose_ = new_pose;
}

void OuterLoopController::update_target(const Prediction& new_target)
{

}

void OuterLoopController::runStateMachine() {
    while (true) {
        switch(curr_state_) {
            case idle:
                if (pred_.ball_in_range_time_ != numeric_limits<int64_t>::max())
                {
                    curr_state_ = prepare;
                }
                if (cur_pose_[0] != 0.0 || cur_pose_[1] != 0.0) 
                {
                    run_idle();
                }
                break;
            case prepare:
                run_prepare();
                break;
            case punch:
                run_punch();
                break;
        }
        sleep_for(milliseconds(1));
    }
}

void OuterLoopController::run_idle() {
    // go to home pose
    std::cout << "[Idle State]: go to " << "(0.0, 0.0, 0.0)" << std::endl;
    publishPlan(Vector3d(0.0, 0.0, 0.0));
    update_curpose(Vector3d(0.0, 0.0, 0.0)); 
}

bool diff(const Vector3d& p1, const Vector3d& p2) {
    if ((p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.x()) * (p1.y() - p2.y()) > 0.01)
        return true;
    return false;
}

Vector3d OuterLoopController::calc_prep_pose(const Prediction& pred) {
    if (pred.ball_inrange_position_[0] < 0) 
    {
        mode_ = 2;
        return Vector3d(-0.15, 0.08, 0);
    }
    mode_ = 1;
    return Vector3d(0.15, 0.08, 0);
}

Vector3d OuterLoopController::calc_punch_pose(const Prediction& pred) {
    if (pred.ball_inrange_position_[0] < 0)
        return Vector3d(0.08, 0.15, 0);
    return Vector3d(-0.08, 0.15, 0);
}

void OuterLoopController::run_prepare() {
    Prediction curr_pred = pred_;

    if (curr_pred.utime_ == prev_pred_time_) {
        return;
    }
    else
    {
        prev_pred_time_ = curr_pred.utime_;
    }

    Vector3d target_pose = calc_prep_pose(curr_pred);
    if (diff(target_pose, cur_pose_)) {
        std::cout << "[Prepare State]: go to (" << target_pose.x() << ", " << 
        target_pose.y() << ", " << target_pose.z() << ")" << std::endl;
        publishPlan(target_pose);
        update_curpose(target_pose);
    }
    microseconds curr_time = duration_cast<microseconds>(
                std::chrono::system_clock::now().time_since_epoch());
    microseconds time_diff = microseconds(curr_pred.ball_in_range_time_) - curr_time;
    if (duration_cast<milliseconds>(time_diff).count() < 400) {
        curr_state_ = punch;
    }
}

void OuterLoopController::run_punch() {
    Prediction curr_pred = pred_;
    Vector3d tar_pose = calc_punch_pose(curr_pred);
    std::cout << "[Punch State]: go to (" << tar_pose.x() << ", " << 
        tar_pose.y() << ", " << tar_pose.z() << ")" << std::endl;
    publishPlan(tar_pose);
    update_curpose(tar_pose);
    curr_state_ = idle; 
}

void OuterLoopController::publishPlan(const Vector3d& arm_pose)
{
    arm_path_t path;
    path.waypoints_num = 1;
    std::vector<std::vector< double > > waypoints( 1, std::vector<double>(2, 0));
    // Flip X and Y for Arm Coordinate system
    waypoints[0][0] = arm_pose[0];
    waypoints[0][1] = arm_pose[1];
    // waypoints[0][2] = arm_pose[2];
    // TODO: mode
    waypoints[0][2] = mode_;
    path.waypoints = waypoints;
    path.speed = 1.0;
    // path.angles_num = 1;
    // std::vector<std::vector< double > > angles( 1, std::vector<double>(4, 0));
    // path.angles = angles;
    lcm_ptr_->publish(channel::ARM_PATH, &path );
    // path.waypoints[0][0] = pr.second.x;
    // path.waypoints[0][1] = pr.second.y;
}