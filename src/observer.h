#ifndef OBSERVER_H
#define OBSERVER_H

#include <iostream>
#include <yaml-cpp/yaml.h>

#include "networks/sender.h"
#include "networks/receiver.h"
#include "planner/Dstar.h"
#include "planner/DWA.h"

class Observer {
    public:
        Observer(const YAML::Node& config);
        ~Observer();

        void update();
        void waitForReceiver();

    private:
        const YAML::Node& conf;

        Sender sender;
        Receiver receiver;
        Dstar dstar;
        DWA dwa;

        Robot* ourRobots;
        Robot* enemyRobots;

        list<state>* dstarPlans;
};

#endif // OBSERVER_H