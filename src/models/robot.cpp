#include "robot.h"

Robot::Robot()
    : confidence(0.0),
      robotId(0),
      x(0.0),
      y(0.0),
      orientation(0.0),
      pixelX(0.0),
      pixelY(0.0),
      height(0.0) {
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

    velocity.x = (robot.x() - pre_x) / deltaTime;
    velocity.y = (robot.y() - pre_y) / deltaTime;
    angularVelocity = (robot.orientation() - pre_orientation) / deltaTime;
    pre_x = robot.x();
    pre_y = robot.y();
    pre_orientation = robot.orientation();
}