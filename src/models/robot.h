// Robot.h
#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include "ssl_vision_detection.pb.h"

using namespace std;

#define DECREASE_VALUE 1e-3

class Robot {
public:
    Robot();
    void update(SSL_DetectionRobot robot, float deltaTime);

    float confidence;
    unsigned int robotId;
    float orientation;
    float pixelX;
    float pixelY;
    float height;

    bool active;
    Eigen::Vector2d pos;
    Eigen::Vector2d dest;
    Eigen::Vector2d vel;
    Eigen::Vector2d prePos;
    float velocity;
    float angularVelocity;
    float preOrientation;
};
