#include "observer.h"

Observer::Observer() : sender(), receiver()
{
    receiver.start();
    preBlueRobots = new Robot[16];
    preYellowRobots = new Robot[16];
    for (int i = 0; i < 16; ++i) {
        preBlueRobots[i] = Robot();
        preYellowRobots[i] = Robot();
    }
}

Observer::~Observer() 
{
    receiver.stop();
}

void Observer::update()
{
    blueRobots = receiver.getBlueRobots();
    yellowRobots = receiver.getYellowRobots();
    float fps = receiver.fps;
    
    for (int i = 0; i < 1; i++) {
        blueRobots[i].velocity.x = (blueRobots[i].x - preBlueRobots[i].x) * fps;
        blueRobots[i].velocity.y = (blueRobots[i].y - preBlueRobots[i].y) * fps;
        // std::cout << "Blue Robot " << i << ": "
        //           << "angularVelocity=" << blueRobots[i].angularVelocity
        //           << ", previous angularVelocity=" << preBlueRobots[i].angularVelocity
        //           << std::endl;

        double deltaAngle = blueRobots[i].orientation - preBlueRobots[i].orientation;

        // ラジアンのラップアラウンド補正（-π?πに収める）
        if (deltaAngle > M_PI) deltaAngle -= 2 * M_PI;
        if (deltaAngle < -M_PI) deltaAngle += 2 * M_PI;

        blueRobots[i].angularVelocity = deltaAngle * fps * (180.0 / M_PI) ;

        // std::cout << "velocity of blue robot " << i << ": "
        //           << "x=" << blueRobots[i].velocity.x
        //           << ", y=" << blueRobots[i].velocity.y
        //           << ", angular=" << blueRobots[i].angularVelocity
        //           << std::endl;
        // std::cout << "Pre Blue Robot " << i << ": "
        //           << "x=" << preBlueRobots[i].x
        //           << ", y=" << preBlueRobots[i].y
        //           << ", orientation=" << preBlueRobots[i].orientation
        //           << ", confidence=" << preBlueRobots[i].confidence
        //           << std::endl;
        preBlueRobots[i] = blueRobots[i];
    }
    // // pre 
    // for (int i = 0; i < 1; i++) {
    //     std::cout << "Blue Robot " << i << ": "
    //                 << "x=" << blueRobots[i].x
    //                 << ", y=" << blueRobots[i].y
    //                 << ", orientation=" << blueRobots[i].orientation
    //                 << ", confidence=" << blueRobots[i].confidence
    //                 << std::endl;
    // }
}