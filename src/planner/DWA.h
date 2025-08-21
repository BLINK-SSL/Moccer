#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <thread>
#include <atomic>
#include <yaml-cpp/yaml.h>

#include "../models/state.h"
#include "../models/robot.h"
#include "../models/cmd.h"

using namespace std;

#define INT_MAX 1000000

class Bot_Model {
public:
    double Max_Velocity;
    double Max_Angular_Velocity;
    double Max_Velocity_Acceleration;
    double Max_Angular_Acceleration;
};

class Pair {
public:
    double Target_Velocity;
    double Target_Angular_Velocity;
};

class Node {
public:
    double Dist_To_Obstacle;
    double Dist_To_Goal;
    double Velocity;
    double Angular_Velocity;
    double VELOCITY;
    double ANGULAR_VELOCITY;
    double Dist_To_D_Star;
};

class DWA {
public:
    DWA(const YAML::Node& config);
    ~DWA();

    void update(Robot* ourRobots, Robot* enemyRobots, vector<Eigen::Vector2d>* dstarPlans);
    void start();
    void stop();

    bool Get_Trajectory(Robot bot);
    double Get_Dist_To_Obstacle();

    double MIN(double a, double b) {
        return a > b ? b : a;
    }

    double MAX(double a, double b) {
        return a < b ? b : a;
    }

    double Calc_Dist(Eigen::Vector2d a, Eigen::Vector2d b) {
        return sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2));
    }

    bool Legal_Vector2d(Eigen::Vector2d x) {
        return true;
    }

    double Get_Dist_To_Goal(Eigen::Vector2d Car_Destination) {
        return Calc_Dist(Trajectory[Trajectory.size() - 1], Car_Destination);
    }

    int Get_H(Eigen::Vector2d Left, Eigen::Vector2d Right) {
        return abs(Left.x() - Right.x()) + abs(Left.y() - Right.y());
    }

    static bool cmp(Eigen::Vector2d a, Eigen::Vector2d b) {
        return a.x() < b.x();
    }

    double Get_D_Star_Dist(std::vector<Eigen::Vector2d> D_Star_Road, Eigen::Vector2d End_Road) {
        double Dist = INT_MAX;
        for (auto &i: D_Star_Road) {
            double Temp = Get_H(End_Road, i);
            if (Temp < Dist) {
                Dist = Temp;
            }
        }
        return Dist;
    }

    void Refresh_Programme() {
        Obstacle_Set.clear();
        Ok_List.clear();

        MIN_Dist_To_Obstacle = 1e100;
        MAX_Dist_To_Obstacle = -10;
        MIN_Dist_To_Goal = 1e100;
        MAX_Dist_To_Goal = -10;
        MIN_Velocity = 1e100;
        MAX_Velocity = -1e100;
        MIN_Angular_Velocity = 1e100;
        MAX_Angular_Velocity = -1e100;
        MIN_Dist_To_D_Star = 1e100;
        MAX_Dist_To_D_Star = -10;
    }

    void run();
    void trajectory(vector<Eigen::Vector2d> dstarPlan, Robot bot);

    RobotCmd* getDwa();

private:
    std::mutex dwaMutex;
    const YAML::Node& conf;

    vector<Eigen::Vector2d> Obstacle_Set;
    vector<Node> Ok_List;
    vector<Eigen::Vector2d> Trajectory;

    std::thread dwaThread_;
    std::atomic<bool> running_;
    float dRatio;

    int maxRobotCount;
    Robot ourRobots[16];
    Robot enemyRobots[16];
    RobotCmd robotCmds[16];
    vector<Eigen::Vector2d> dstarPlans[16];

    double MIN_Dist_To_Obstacle;
    double MAX_Dist_To_Obstacle;
    double MIN_Dist_To_Goal;
    double MAX_Dist_To_Goal;
    double MIN_Velocity;
    double MAX_Velocity;
    double MIN_Angular_Velocity;
    double MAX_Angular_Velocity;
    double MIN_Dist_To_D_Star;
    double MAX_Dist_To_D_Star;

    float maxVelocity;
    float maxVelocityAcc;
    float velocityAccuracy;

    float delta;
    float predictDelta;

    int One_Block;

    float Goal;
    float Velocity;
    float D_Star;
};