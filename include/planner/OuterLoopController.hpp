#pragma once

#include <Eigen/Core>

class OuterLoopController
{
public:
    enum state {idle, prepare, punch};
    // (x, y, wrist_angle)
    Eigen::Vector3d cur_pose(-1, -1, 0);
    Prediction prev_pred;
    int mode;
private:
    void update_target(const Prediction& new_target);

    void update_curpose(const Eigen::Vector3d& new_pose);

    void runStateMachine();

    void idle();

    bool diff(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

    Eigen::Vector3d calc_prep_pose(const Prediction& pred);

    Eigen::Vector3d calc_punch_pose(const Prediction& pred);

    void prepare();

    void punch();
};