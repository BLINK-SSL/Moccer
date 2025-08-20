#pragma once

#include <eigen3/Eigen/Dense>

struct RobotCmd {
    int id;
    Eigen::Vector2d vel;
    double angVel;
    double kickPow;
    double chipKickPow;
    double dribblePow;
};