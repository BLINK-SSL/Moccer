#pragma once
#include <iostream>
#include<vector>
#include<cmath>
#include<cstring>
#include<algorithm>

#include "../models/robot.h"

using namespace std;

#define Max_Range 120
#define Delta 0.016
#define Predict_Delta 0.1
#define Velocity_Accuracy 1
#define Angular_Velocity_Accuracy 0.1
#define One_Block 1.0
#define Safe_Distance 200.0 //安全距離
#define Alpha 0.5 //Obstacle
#define Beta (-1) //Goal
#define Gamma 1.0 //Velocity
#define INT_MAX 1000000000
#define Delta2 (-1) //Dist_To_D_Star

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

class DWAPlanner {
public:
    DWAPlanner() {
    }
    ~DWAPlanner() {
    }

    void Init(Bot_Model bot_model, Coordinate car_coordinate, Coordinate car_destination);
    void Set_Obstacle_Set(vector<Coordinate> obstacle_set);
    void Get_Answer(Pair &answer);

    double MIN(double a, double b) {
        return a > b ? b : a;
    }

    double MAX(double a, double b) {
        return a < b ? b : a;
    }

    double Calc_Dist(Coordinate a, Coordinate b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    // bool Legal_Coordinate(Coordinate x) {
    //     std::cout<< "Checking coordinate: " << x.x << ", " << x.y << std::endl;
    //     if (x.x < 0 || x.x >= Max_Range || x.y < 0 || x.y >= Max_Range) return false;
    //     else return true;
    // }
    bool Legal_Coordinate(Coordinate x) {
            return true;
    }

    bool Get_Trajectory(Coordinate Car_Coordinate, double Now_Velocity, double Now_Angle, double Now_Angular_Velocity) {
        double Car_x = One_Block * Car_Coordinate.x;
        double Car_y = One_Block * Car_Coordinate.y;
        double Time_Sum = 0;
        while (Time_Sum <= Predict_Delta) {
            Time_Sum += Delta;
            double Next_Angle = Now_Angle + Now_Angular_Velocity * Delta;
            Car_x += Now_Velocity * cos(Next_Angle) * Delta;
            Car_y += Now_Velocity * sin(Next_Angle) * Delta;
            Now_Angle = Next_Angle;
            if (Legal_Coordinate({lround(Car_x / One_Block), lround(Car_y / One_Block)})) {
                // std::cout << "Time: " << Time_Sum
                //           << ", Car position: " << lround(Car_x / One_Block) << ", " << lround(Car_y / One_Block) << std::endl;
                // std::cout << "Adding trajectory point: " << lround(Car_x / One_Block) << ", " << lround(Car_y / One_Block) << std::endl;
                Trajectory.push_back({lround(Car_x / One_Block), lround(Car_y / One_Block)});
            } else {
                return false;
            }
        }
        if (Trajectory.empty()) return false;
        return true;
    }

    double Get_Dist_To_Obstacle() {
        if (Obstacle_Set.empty()) {
            return -1;
        }
        double minn = 1e100;
        for (auto &i: Trajectory) {
            for (auto &j: Obstacle_Set) {
                double Dist = Calc_Dist(i, j);
                minn = MIN(minn, Dist);
            }
        }
        return minn;
    }

    double Get_Dist_To_Goal(Coordinate Car_Destination) {
        return Calc_Dist(Trajectory[Trajectory.size() - 1], Car_Destination);
    }

    int Get_H(Coordinate Left, Coordinate Right) {
        return abs(Left.x - Right.x) + abs(Left.y - Right.y);
    }

    static bool cmp(Coordinate a, Coordinate b) {
        // 比較ロジック
        return a.x < b.x; // 例
    }

    double Get_D_Star_Dist(std::vector<Coordinate> D_Star_Road, Coordinate End_Road) {
        double Dist = INT_MAX;
        for (auto &i: D_Star_Road) {
            double Temp = Get_H(End_Road, i);//和求h的估??通用，都是坐?距?
            if (Temp < Dist) {//取点到点最小距?作?点到?的距?
                Dist = Temp;
            }
        }
        // std::cout << "Distance: " << Dist << std::endl;
        return Dist;
    }



    void Refresh_Programme() {
        Obstacle_Set.clear();
        Ok_List.clear();
    }

    Pair DWA(std::vector<Coordinate> D_Star_Road, Coordinate Car_Coordinate, double Now_Angle, double Now_Velocity,
             double Now_Angular_Velocity, Coordinate Car_Destination, Bot_Model Model, Robot* blueRobots, Robot* yellowRobots) {
        // std::cout << "coordinate: " << Car_Coordinate.x << ", " << Car_Coordinate.y << std::endl;
        Refresh_Programme();
        // for (auto &robot: blueRobots.begin()) {
        for (int i = 0; i < 11; ++i) {
            Obstacle_Set.push_back({static_cast<int>(yellowRobots[i].x / One_Block), static_cast<int>(yellowRobots[i].y / One_Block)});
        }
        double MIN_Dist_To_Obstacle = 1e100;
        double MAX_Dist_To_Obstacle = -10;
        double MIN_Dist_To_Goal = 1e100;
        double MAX_Dist_To_Goal = -10;
        double MIN_Velocity = 1e100;
        double MAX_Velocity = -1e100;
        double MIN_Angular_Velocity = 1e100;
        double MAX_Angular_Velocity = -1e100;
        double MIN_Dist_To_D_Star = 1e100;
        double MAX_Dist_To_D_Star = -10;
        for (double Velocity = Now_Velocity - Predict_Delta * Model.Max_Velocity_Acceleration;
             Velocity <= Now_Velocity + Predict_Delta * Model.Max_Velocity_Acceleration; Velocity += Velocity_Accuracy) {
            for (double Angular_Velocity = Now_Angular_Velocity - Predict_Delta * Model.Max_Angular_Acceleration;
                 Angular_Velocity <= Now_Angular_Velocity + Predict_Delta *
                                                            Model.Max_Angular_Acceleration; Angular_Velocity += Angular_Velocity_Accuracy) {
                if (fabs(Velocity) > Model.Max_Velocity || fabs(Angular_Velocity) > Model.Max_Angular_Velocity){
                    continue;
                }
                Trajectory.clear();
                if (!Get_Trajectory(Car_Coordinate, Velocity, Now_Angle, Angular_Velocity)) continue;

                double Break_Length = ((Velocity * Velocity / (2 * Model.Max_Velocity_Acceleration)) / One_Block);
                double Dist_To_Obstacle = Get_Dist_To_Obstacle();
                // if (Dist_To_Obstacle != -1 && Dist_To_Obstacle < Safe_Distance) continue;
                
                // if (Dist_To_Obstacle != -1 && Break_Length > Dist_To_Obstacle) {
                //     std::cout << "Brake Length: " << Break_Length << ", Dist To Obstacle: " << Dist_To_Obstacle << std::endl;
                //     continue;
                // }
                double Dist_To_Goal = Get_Dist_To_Goal(Car_Destination);
                // std::cout << "Dist to goal: " << Dist_To_Goal << std::endl;
                double Dist_To_D_Star = Get_D_Star_Dist(D_Star_Road, Trajectory[Trajectory.size() - 1]);
                MIN_Dist_To_Obstacle = MIN(Dist_To_Obstacle, MIN_Dist_To_Obstacle);
                MAX_Dist_To_Obstacle = MAX(Dist_To_Obstacle, MAX_Dist_To_Obstacle);
                MIN_Dist_To_Goal = MIN(Dist_To_Goal, MIN_Dist_To_Goal);
                MAX_Dist_To_Goal = MAX(Dist_To_Goal, MAX_Dist_To_Goal);
                MIN_Velocity = MIN(Velocity, MIN_Velocity);
                MAX_Velocity = MAX(Velocity, MAX_Velocity);
                MIN_Angular_Velocity = MIN(Angular_Velocity, MIN_Angular_Velocity);
                MAX_Angular_Velocity = MAX(Angular_Velocity, MAX_Angular_Velocity);
                MIN_Dist_To_D_Star = MIN(Dist_To_D_Star, MIN_Dist_To_D_Star);
                MAX_Dist_To_D_Star = MAX(Dist_To_D_Star, MAX_Dist_To_D_Star);

                double angle_to_goal = atan2(Car_Destination.y - Car_Coordinate.y,
                                            Car_Destination.x - Car_Coordinate.x);
                // double angle_diff = fabs(Normalize_Angle(angle_to_goal - Now_Angle));

                Ok_List.push_back(
                        {Dist_To_Obstacle, Dist_To_Goal, Velocity, Angular_Velocity, Velocity, Angular_Velocity,
                         Dist_To_D_Star});
            }
        }

        for (auto &i : Ok_List) {
            i.Dist_To_Obstacle =
                (i.Dist_To_Obstacle - MIN_Dist_To_Obstacle) / (MAX_Dist_To_Obstacle - MIN_Dist_To_Obstacle + 1e-8);
            i.Dist_To_Goal =
                (i.Dist_To_Goal - MIN_Dist_To_Goal) / (MAX_Dist_To_Goal - MIN_Dist_To_Goal + 1e-8);
            i.Velocity =
                (i.Velocity - MIN_Velocity) / (MAX_Velocity - MIN_Velocity + 1e-8);
            i.Angular_Velocity =
                (i.Angular_Velocity - MIN_Angular_Velocity) / (MAX_Angular_Velocity - MIN_Angular_Velocity + 1e-8);
            i.Dist_To_D_Star =
                (i.Dist_To_D_Star - MIN_Dist_To_D_Star) / (MAX_Dist_To_D_Star - MIN_Dist_To_D_Star + 1e-8);
        }

        double MAX_Score = -1e100;
        Pair Target = {0, 0};

        for (auto &i : Ok_List) {
            double Now_Score = 0;
            Now_Score += Beta * i.Dist_To_Goal;
            Now_Score += Gamma * i.Velocity;
            // Now_Score += Alpha * i.Dist_To_Obstacle;
            Now_Score += Delta2 * i.Dist_To_D_Star;

            if (Now_Score > MAX_Score) {
                MAX_Score = Now_Score;
                Target.Target_Angular_Velocity = i.ANGULAR_VELOCITY;
                Target.Target_Velocity = i.VELOCITY;
            }
        }
        return Target;
    }

private:
    vector<Coordinate> Obstacle_Set;
    vector<Node> Ok_List;
    vector<Coordinate> Trajectory;
};