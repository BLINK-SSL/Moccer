#include "robot.h"

Robot::Robot()
    : confidence(0.0),
      robotId(0),
      x(0.0),
      y(0.0),
      orientation(0.0),
      pixelX(0.0),
      pixelY(0.0),
      height(0.0),
      pos({0.0, 0.0}),
      dest({0.0, 0.0}),
      vel({0.0, 0.0}),
      velocity(0.0),
      angularVelocity(0.0),
      pre_x(0.0),
      pre_y(0.0),
      pre_orientation(0.0),
      active(false) {
}

void Robot::update(SSL_DetectionRobot robot, float deltaTime) {
    confidence = robot.confidence();
    x = robot.x();
    y = robot.y();
    pixelX = robot.pixel_x();
    pixelY = robot.pixel_y();
    if (robot.has_robot_id()) robotId = robot.robot_id();
    if (robot.has_orientation()) orientation = robot.orientation();
    if (robot.has_height()) height = robot.height();

    active = (robot.x() != 0 && robot.y() != 0);

    if (deltaTime <= 0) {
        return;
    }
    if (robot.x() - pre_x != 0) {
        vel.x() = (robot.x() - pre_x) / deltaTime;
    }
    if (robot.y() - pre_y != 0) {
        vel.y() = (robot.y() - pre_y) / deltaTime;
    }

    velocity = sqrt(vel.x() * vel.x() + vel.y() * vel.y());
    angularVelocity = (robot.orientation() - pre_orientation) / deltaTime;
    pre_x = robot.x();
    pre_y = robot.y();
    pre_orientation = robot.orientation();

    pre_orientation = robot.orientation();
}