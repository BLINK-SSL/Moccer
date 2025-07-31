#pragma once

#include <iostream>
#include <cmath> 

#define ROBOT_RADIUS 180

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
        return std::hypot(x - px, y - py) < (radius*2);
    }
};

class RectangleObstacle : public Obstacle {
public:
    double x, y, width, height;
    RectangleObstacle(double x_, double y_, double width_, double height_)
        : x(x_), y(y_), width(width_), height(height_) {}
    bool collides(double px, double py) const override {
        double half_w = width / 2.0 + ROBOT_RADIUS;
        double half_h = height / 2.0 + ROBOT_RADIUS;

        return std::abs(px - x) <= half_w && std::abs(py - y) <= half_h;
    }
};