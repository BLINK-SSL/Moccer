#include "planner.h"

Planner::Planner(const YAML::Node& config, int id) : conf(config), id(id), dstar(config), dwa(config) {
}

Planner::~Planner() {
}

void Planner::start() {
    planThread_ = std::thread(&Planner::run, this);
}

void Planner::stop() {
    running_ = false;
    if (planThread_.joinable()) {
        planThread_.join();
    }
}

void Planner::update(Robot* ourRobots, Robot* enemyRobots) {
    std::lock_guard<std::mutex> lock(planMutex);
    running_ = true;
    this->ourRobots = ourRobots;
    this->enemyRobots = enemyRobots;
}

RobotCmd Planner::getCmd() {
    return cmd;
}

void Planner::run() {
    while (!running_);
    while (running_) {
        vector<Eigen::Vector2d> dstarPlan = dstar.run(ourRobots, enemyRobots, id);
        cmd = dwa.run(ourRobots, enemyRobots, id, dstarPlan);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
