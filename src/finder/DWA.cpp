#include "DWA.h"

DWA::DWA() {
    running_ = false;
}

DWA::~DWA() {
    stop();
}

void DWA::start() {
    running_ = true;
    dwaThread_ = std::thread(&DWA::run, this);
}

void DWA::stop() {
    running_ = false;
    if (dwaThread_.joinable()) {
        dwaThread_.join();
    }
}

double DWA::Get_Dist_To_Obstacle() {
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

// bool Legal_Coordinate(Coordinate x) {
//     std::cout<< "Checking coordinate: " << x.x << ", " << x.y << std::endl;
//     if (x.x < 0 || x.x >= Max_Range || x.y < 0 || x.y >= Max_Range) return false;
//     else return true;
// }

bool DWA::Get_Trajectory(Coordinate Car_Coordinate, double Now_Velocity, double Now_Angle, double Now_Angular_Velocity) {
    double Car_x = One_Block * Car_Coordinate.x;
    double Car_y = One_Block * Car_Coordinate.y;
    double Time_Sum = 0;
    while (Time_Sum <= Predict_Delta) {
        Time_Sum += Delta;
        double Next_Angle = Now_Angle + Now_Angular_Velocity * Delta;
        Car_x += Now_Velocity * cos(Next_Angle) * Delta;
        Car_y += Now_Velocity * sin(Next_Angle) * Delta;
        Now_Angle = Next_Angle;
        if (Legal_Coordinate({static_cast<int>(lround(Car_x / One_Block)), static_cast<int>(lround(Car_y / One_Block))})) {
            // std::cout << "Time: " << Time_Sum
            //           << ", Car position: " << lround(Car_x / One_Block) << ", " << lround(Car_y / One_Block) << std::endl;
            // std::cout << "Adding trajectory point: " << lround(Car_x / One_Block) << ", " << lround(Car_y / One_Block) << std::endl;
            Trajectory.push_back({static_cast<int>(Car_x / One_Block), static_cast<int>(Car_y / One_Block)});
        } else {
            // return false;
        }
    }
    // if (Trajectory.empty()) return false;
    return true;
}

void DWA::run() {
    while (running_) {
        std::vector<Coordinate> D_Star_Road;
        for (auto &coord : path) {
            D_Star_Road.push_back({static_cast<int>(coord.x * dRatio), static_cast<int>(coord.y * dRatio)});
        }
        Coordinate s_start_coord = {static_cast<int>(s_start.x * dRatio), static_cast<int>(s_start.y * dRatio)};
        Coordinate s_goal_coord = {static_cast<int>(s_goal.x * dRatio), static_cast<int>(s_goal.y * dRatio)};
        Bot_Model bot_model = {1000.0, 60}; // 60 degrees in radians
        float velocity = sqrt(blueRobots[0].velocity.x * blueRobots[0].velocity.x + blueRobots[0].velocity.y * blueRobots[0].velocity.y);
        _pair = dwaPlanner.DWA(D_Star_Road, s_start_coord, blueRobots[0].orientation, velocity, blueRobots[0].angularVelocity, s_goal_coord, bot_model, blueRobots, yellowRobots);
        std::cout << "Robot velocity: " << blueRobots[0].velocity.x << ", " << blueRobots[0].velocity.y << std::endl;
        // std::cout << blueRobots[0].x << ", " << blueRobots[0].y << std::endl;/*  */
        // std::cout << "Target velocity: " << _pair.Target_Velocity << ", Target Angular Velocity: " << _pair.Target_Angular_Velocity << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}