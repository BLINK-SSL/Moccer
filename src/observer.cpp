#include "observer.h"

Observer::Observer(const YAML::Node& config)
    : conf(config), sender(config), receiver(config)
{
    receiver.start();
    maxRobotCount = conf["General"]["MaxRobotCount"].as<int>();
    planners.reserve(maxRobotCount);

    for (int i = 0; i < maxRobotCount; ++i) {
        planners.push_back(std::make_unique<Planner>(conf, i));
        planners[i]->start();
    }
}

Observer::~Observer() {
    for (auto& p : planners) p->stop();
    receiver.stop();
}

void Observer::update() {
    // Safely copy the latest frame into member variables
    {
        lock_guard<mutex> lock(mtx);
        ourRobots = receiver.getOurRobots();
        enemyRobots = receiver.getEnemyRobots();
    }

    // Update planners and collect commands
    vector<RobotCmd> cmds(maxRobotCount);
    for (int i = 0; i < maxRobotCount; ++i) {
        planners[i]->update(ourRobots, enemyRobots);
        cmds[i] = planners[i]->getCmd();
    }

    // Send commands via the network
    sender.send(false, cmds.data());
}
