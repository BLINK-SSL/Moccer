// Robot.h
#pragma once

#include "ssl_vision_detection.pb.h"

struct Point {
    float x, y;
};

class Robot {
public:
    Robot();
    void update(SSL_DetectionRobot robot);

    float confidence;
    unsigned int robotId;
    float x;
    float y;
    float orientation;
    float pixelX;
    float pixelY;
    float height;

    Point velocity;
    float angularVelocity;

};
