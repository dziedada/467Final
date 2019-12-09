#include <planner/OuterLoopController.hpp>

using Eigen::Vector2d;
using Eigen::Vector3d;


void OuterLoopController::update_curpose(const Vector3d& new_pose) {
    cur_pose = new_pose;
}

void OuterLoopController::runStateMachine() {
    while (true) {
        switch(state) {
            case idle:
                if (balls.size()) {
                    state = prepare;
                }
                if (cur_pose.x() != 0.0 || cur_pose.y() != 0.0) {
                    idle();
                }
                break;
            case prepare:
                prepare();
                break;
            case punch:
                punch();
                break;
        }
        usleep(1000);
    }
}

void OuterLoopController::idle() {
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

Vector3d calc_prep_pose(const Prediction& pred) {
    if (pred.arrival.x() < 0) {
        mode = 2;
        return Vector3d(-0.15, 0.08, 0);
    }
    mode = 1;
    return Vector3d(0.15, 0.08, 0)
}

Vector3d calc_punch_pose(const Prediction& pred) {
    if (pred.arrival.x() < 0)
        return Vector3d(0.08, 0.15, 0);
    return Vector3d(-0.08, 0.15, 0)
}

void OuterLoopController::prepare() {
    Prediction pred;
    pred.time = DBL_MAX;
    for (auto &ball: balls) {
        Prediction new_pred = ball.getPred();
        if (new_pred.time < pred.time) {
            pred = new_pred;
        }
    }

    if (pred == prev_pred) {
        return;
    }

    Vector3d tar_pose = calc_prep_pose(pred);
    if (diff(tar_pose, cur_pose)) {
        std::cout << "[Prepare State]: go to (" << tar_pose.x() << ", " << 
        tar_pose.y() << ", " << tar_pose.z() << ")" << std::endl;
        publishPlan(tar_pose);
        update_curpose(tar_pose);
    }
    if (pred.time < 0.5) {
        state = punch;
    }
}

void OuterLoopController::punch() {
    std::cout << "[Punch State]: go to (" << tar_pose.x() << ", " << 
        tar_pose.y() << ", " << tar_pose.z() << ")" << std::endl;
    Vector3d tar_pose = calc_punch_pose(pred);
    publishPlan(tar_pose);
    update_curpose(tar_pose);
    state = idle; 
}