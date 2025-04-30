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

void Robot::update(SSL_DetectionRobot robot) {
    confidence = robot.confidence();
    x = robot.x();
    y = robot.y();
    pixelX = robot.pixel_x();
    pixelY = robot.pixel_y();
    if (robot.has_robot_id()) robotId = robot.robot_id();
    if (robot.has_orientation()) orientation = robot.orientation();
    if (robot.has_height()) height = robot.height();
}