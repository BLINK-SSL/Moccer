#pragma once

#include <iostream>
#include <yaml-cpp/yaml.h>

#include "networks/sender.h"
#include "networks/receiver.h"
#include "planner/planner.h"

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
        std::vector<std::unique_ptr<Planner>> planners;

        Robot* ourRobots;
        Robot* enemyRobots;

        int maxRobotCount;
};
