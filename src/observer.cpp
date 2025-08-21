#include "observer.h"

Observer::Observer(const YAML::Node& config) : conf(config), sender(config), receiver(config), dstar(config), dwa(config) {
    receiver.start();
    dstar.start();
    dwa.start();
}

Observer::~Observer() {
    receiver.stop();
    dstar.stop();
    dwa.stop();
}

void Observer::waitForReceiver() {
    while (!receiver.isNewWorld);
    receiver.isNewWorld = false;
}

void Observer::update() {
    waitForReceiver();

    ourRobots = receiver.getOurRobots();
    enemyRobots = receiver.getEnemyRobots();

    dstar.update(ourRobots, enemyRobots);
    dstarPlans = dstar.getPlans();

    dwa.update(ourRobots, enemyRobots, dstarPlans);
    dwaPlans = dwa.getDwa();

    sender.send(false, dwaPlans);
}