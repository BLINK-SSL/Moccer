#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <thread>
#include <atomic>

#include "../models/robot.h"

using namespace std;

#define Delta 0.01
#define Predict_Delta 0.1
#define Velocity_Accuracy 100
#define Angular_Velocity_Accuracy 1
#define One_Block 1.0
#define Safe_Distance 200.0
#define Alpha 0.5 //Obstacle
#define Beta (-1) //Goal
#define Gamma 1.0 //Velocity
#define INT_MAX 1000000000
#define Delta2 (0) //Dist_To_D_Star

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

class Coordinate {
public:
    int x;
    int y;
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
    DWA();
    ~DWA();

    void start();
    void stop();

    bool Get_Trajectory(Coordinate Car_Coordinate, double Now_Velocity, double Now_Angle, double Now_Angular_Velocity);
    double Get_Dist_To_Obstacle();

    double MIN(double a, double b) {
        return a > b ? b : a;
    }

    double MAX(double a, double b) {
        return a < b ? b : a;
    }

    double Calc_Dist(Coordinate a, Coordinate b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    bool Legal_Coordinate(Coordinate x) {
        return true;
    }

    double Get_Dist_To_Goal(Coordinate Car_Destination) {
        return Calc_Dist(Trajectory[Trajectory.size() - 1], Car_Destination);
    }

    int Get_H(Coordinate Left, Coordinate Right) {
        return abs(Left.x - Right.x) + abs(Left.y - Right.y);
    }

    static bool cmp(Coordinate a, Coordinate b) {
        return a.x < b.x;
    }

    double Get_D_Star_Dist(std::vector<Coordinate> D_Star_Road, Coordinate End_Road) {
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
    }

    void run();

private:
    vector<Coordinate> Obstacle_Set;
    vector<Node> Ok_List;
    vector<Coordinate> Trajectory;

    std::thread dwaThread_;
    std::atomic<bool> running_;
};