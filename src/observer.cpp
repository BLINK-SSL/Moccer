#include "observer.h"

Observer::Observer() : sender(), receiver(), dstar()
{
    receiver.start();
    dstar.start();
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

void Observer::waitForReceiver() {
    while (!receiver.isNewWorld);
    receiver.isNewWorld = false;
}
void Observer::update()
{
    waitForReceiver();

    blueRobots = receiver.getBlueRobots();
    yellowRobots = receiver.getYellowRobots();

    dstar.update(blueRobots, yellowRobots);
    Pair pair = dstar.getPair();
    // std::cout << "Target Velocity: " << pair.Target_Velocity << ", Target Angular Velocity: " << pair.Target_Angular_Velocity << std::endl;
    sender.send(false, pair.Target_Velocity, pair.Target_Angular_Velocity, blueRobots[0].orientation);
}