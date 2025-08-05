#include "observer.h"

Observer::Observer() : sender(), receiver()
{
    receiver.start();
}

Observer::~Observer() 
{
    receiver.stop();
}

void Observer::update()
{
    blueRobots = receiver.getBlueRobots();
    yellowRobots = receiver.getYellowRobots();
    // for (int i = 0; i < 1; i++) {
    //     std::cout << "Blue Robot " << i << ": "
    //                 << "x=" << blueRobots[i].x
    //                 << ", y=" << blueRobots[i].y
    //                 << ", orientation=" << blueRobots[i].orientation
    //                 << ", confidence=" << blueRobots[i].confidence
    //                 << std::endl;
    // }
}