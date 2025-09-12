#include "robot.h"

Robot::Robot()
    : confidence(0.0),
      robotId(0),
      orientation(0.0),
      pixelX(0.0),
      pixelY(0.0),
      height(0.0),
      pos({0.0, 0.0}),
      dest({0.0, 0.0}),
      vel({0.0, 0.0}),
      velocity(0.0),
      angularVelocity(0.0),
      prePos({0.0, 0.0}),
      preOrientation(0.0),
      active(false) {
}

void Robot::update(SSL_DetectionRobot robot, float deltaTime) {
    confidence = robot.confidence();
    pos.x() = robot.x();
    pos.y() = robot.y();
    pixelX = robot.pixel_x();
    pixelY = robot.pixel_y();
    if (robot.has_robot_id()) robotId = robot.robot_id();
    if (robot.has_orientation()) orientation = robot.orientation();
    if (robot.has_height()) height = robot.height();

    active = (robot.x() != 0 && robot.y() != 0);

    if (deltaTime <= 0) {
        return;
    }
    if (robot.x() - prePos.x() != 0) {
        vel.x() = (robot.x() - prePos.x()) / deltaTime;
    } else {
        pos.x() -= DECREASE_VALUE;
    }
    if (robot.y() - prePos.y() != 0) {
        vel.y() = (robot.y() - prePos.y()) / deltaTime;
    } else {
        pos.y() -= DECREASE_VALUE;
    }

    velocity = sqrt(vel.x() * vel.x() + vel.y() * vel.y());
    angularVelocity = (robot.orientation() - preOrientation) / deltaTime;
    prePos = pos;
    preOrientation = robot.orientation();
    int temp = 1;
    if (robotId == 0) {
        dest = Eigen::Vector2d(-4200.0 * temp, 1800.0);
    } else if (robotId == 1) {
        dest = Eigen::Vector2d(-100.0 * temp, 0.0);
    } else if (robotId == 2) {
        dest = Eigen::Vector2d(-3000.0 * temp, 0.0);
    } else if (robotId == 3) {
        dest = Eigen::Vector2d(-2000.0 * temp, 2000.0);
    } else if (robotId == 4) {
        dest = Eigen::Vector2d(-2500.0 * temp, 2000.0);
    } else if (robotId == 5) {
        dest = Eigen::Vector2d(-3500.0 * temp, 3000.0);
    } else if (robotId == 6) {
        dest = Eigen::Vector2d(-4000.0 * temp, 3000.0);
    } else if (robotId == 7) {
        dest = Eigen::Vector2d(-4000.0 * temp, 0.0);
    } else {
        dest = Eigen::Vector2d(-2000.0 * temp, 0.0);
    }
}