// Robot.h
#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include "ssl_vision_detection.pb.h"

using namespace std;

class Robot {
public:
    Robot();
    void update(SSL_DetectionRobot robot, float deltaTime);

    float confidence;
    unsigned int robotId;
    float x;
    float y;
    float orientation;
    float pixelX;
    float pixelY;
    float height;

    bool active;
    Eigen::Vector2d pos;
    Eigen::Vector2d dest;
    Eigen::Vector2d vel;
    float velocity;
    float angularVelocity;
    float pre_x;
    float pre_y;
    float pre_orientation;
};
