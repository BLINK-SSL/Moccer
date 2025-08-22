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

}