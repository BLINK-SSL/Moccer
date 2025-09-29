#pragma once 

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <thread>

#include "../models/robot.h"
#include "Dstar.h"
#include "DWA.h"

class Planner {
public:
    Planner(const YAML::Node& config, int id);
    ~Planner();

    void update(const std::vector<Robot>& ourRobots, const std::vector<Robot>& enemyRobots);
    void start();
    void stop();
    void run();
    
    RobotCmd getCmd();

private:
    const YAML::Node& conf;
    Dstar dstar;
    DWA dwa;
    int id;

    std::thread planThread_;
    std::atomic<bool> running_;
    std::mutex planMutex;

    std::vector<Robot> ourRobots;
    std::vector<Robot> enemyRobots;

    RobotCmd cmd;
};