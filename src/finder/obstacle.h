#pragma once

#include <iostream>

#define ROBOT_RADIUS 0.09

class Obstacle {
public:
    virtual ~Obstacle() = default;
    virtual bool collides(double px, double py) const = 0;
};

class CircleObstacle : public Obstacle {
public:
    double x, y, radius;
    CircleObstacle(double x_, double y_, double radius_)
        : x(x_), y(y_), radius(radius_) {}
    bool collides(double px, double py) const override {
        return std::hypot(x - px, y - py) < (radius + ROBOT_RADIUS);
    }
};