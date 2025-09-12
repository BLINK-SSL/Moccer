#include "observer.h"

Observer::Observer(const YAML::Node& config) : conf(config), sender(config), receiver(config){
    receiver.start();
    maxRobotCount = conf["General"]["MaxRobotCount"].as<int>();
    planners.reserve(maxRobotCount);
    for (int i = 0; i < maxRobotCount; ++i) {
        planners.push_back(std::make_unique<Planner>(conf, i));
        planners[i]->start();
    }

}

Observer::~Observer() {
    receiver.stop();
    for (int i = 0; i < planners.size(); ++i) {
        planners[i]->stop();
    }
}

void Observer::waitForReceiver() {
    while (!receiver.isNewWorld);
    receiver.isNewWorld = false;
}

void Observer::update() {

    waitForReceiver();
    
    ourRobots = receiver.getOurRobots();
    enemyRobots = receiver.getEnemyRobots();

    RobotCmd cmds[maxRobotCount];
    for (int i = 0; i < maxRobotCount; ++i) {
        planners[i]->update(ourRobots, enemyRobots);
        cmds[i] = planners[i]->getCmd();
    }
    sender.send(false, cmds);
}