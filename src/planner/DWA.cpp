#include "DWA.h"

DWA::DWA(const YAML::Node& config): conf(config), running_(false) {
    One_Block = 1;

    dRatio = conf["Planner"]["dRatio"].as<float>();

    maxVelocity = conf["Robot"]["MaxVelocity"].as<float>();
    maxVelocityAcc = conf["Robot"]["MaxVelocityAcceleration"].as<float>();
    velocityAccuracy = conf["Planner"]["VelocityAccuracy"].as<float>();

    delta = conf["Planner"]["Delta"].as<float>();
    predictDelta = conf["Planner"]["PredictDelta"].as<float>();
    
    maxRobotCount = conf["General"]["MaxRobotCount"].as<int>();
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
    while (Time_Sum <= predictDelta) {
        Time_Sum += delta;
        double Next_Angle = Now_Angle + Now_Angular_Velocity * delta;
        Car_x += Now_Velocity * cos(Next_Angle) * delta;
        Car_y += Now_Velocity * sin(Next_Angle) * delta;
        Now_Angle = Next_Angle;
        if (Legal_Coordinate({roundf(Car_x / One_Block), roundf(Car_y / One_Block)})) {
            // std::cout << "Time: " << Time_Sum
            //           << ", Car position: " << lround(Car_x / One_Block) << ", " << lround(Car_y / One_Block) << std::endl;
            // std::cout << "Adding trajectory point: " << lround(Car_x / One_Block) << ", " << lround(Car_y / One_Block) << std::endl;
            Trajectory.push_back({static_cast<float>(Car_x / One_Block), static_cast<float>(Car_y / One_Block)});
        } else {
            // return false;
        }
    }
    // if (Trajectory.empty()) return false;
    return true;
}

void DWA::update(Robot* ourRobots, Robot* enemyRobots, list<state>* dstarPlans) {
    for (int i = 0; i < 16; ++i) {
        this->ourRobots[i] = ourRobots[i];
        this->enemyRobots[i] = enemyRobots[i];
    }
    // this->dstarPlans = dstarPlans;
}

void DWA::run() {
    while (running_) {
        Refresh_Programme();

        for (int i = 0; i < 11; ++i) {
            if (enemyRobots[i].active) {
                Obstacle_Set.push_back({enemyRobots[i].x, enemyRobots[i].y});
            }
        }
        // std::vector<Coordinate> D_Star_Road;
        // for (auto &coord : path) {
        //     D_Star_Road.push_back({static_cast<int>(coord.x * dRatio), static_cast<int>(coord.y * dRatio)});
        // }
        
        // (D_Star_Road, s_start_coord, blueRobots[0].orientation, velocity, blueRobots[0].angularVelocity, s_goal_coord, bot_model, blueRobots, yellowRobots);
        for (int i = 0; i < maxRobotCount; i++) {
            trajectory(dstarPlans[i], ourRobots[i]);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void DWA::trajectory(list<state> dstarPlan, Robot bot) {
    for (double Velocity = bot.velocity - predictDelta * maxVelocityAcc; Velocity <= bot.velocity + predictDelta * maxVelocityAcc; Velocity += velocityAccuracy) {
        if (fabs(Velocity) > maxVelocity) continue;

        Trajectory.clear();
        // if (!Get_Trajectory(bot.pos, Velocity, Now_Angle, Angular_Velocity)) continue;

        // double Break_Length = ((Velocity * Velocity / (2 * Model.Max_Velocity_Acceleration)) / One_Block);
        // double Dist_To_Obstacle = Get_Dist_To_Obstacle();
        // // if (Dist_To_Obstacle != -1 && Dist_To_Obstacle < Safe_Distance) continue;
        // // if (Dist_To_Obstacle != -1 && Break_Length > Dist_To_Obstacle) continue;

        // double Dist_To_Goal = Get_Dist_To_Goal(Car_Destination);
        // double Dist_To_D_Star = Get_D_Star_Dist(D_Star_Road, Trajectory[Trajectory.size() - 1]);
        // MIN_Dist_To_Obstacle = MIN(Dist_To_Obstacle, MIN_Dist_To_Obstacle);
        // MAX_Dist_To_Obstacle = MAX(Dist_To_Obstacle, MAX_Dist_To_Obstacle);
        // MIN_Dist_To_Goal = MIN(Dist_To_Goal, MIN_Dist_To_Goal);
        // MAX_Dist_To_Goal = MAX(Dist_To_Goal, MAX_Dist_To_Goal);
        // MIN_Velocity = MIN(Velocity, MIN_Velocity);
        // MAX_Velocity = MAX(Velocity, MAX_Velocity);
        // MIN_Angular_Velocity = MIN(Angular_Velocity, MIN_Angular_Velocity);
        // MAX_Angular_Velocity = MAX(Angular_Velocity, MAX_Angular_Velocity);
        // MIN_Dist_To_D_Star = MIN(Dist_To_D_Star, MIN_Dist_To_D_Star);
        // MAX_Dist_To_D_Star = MAX(Dist_To_D_Star, MAX_Dist_To_D_Star);

        // double angle_to_goal = atan2(Car_Destination.y - Car_Coordinate.y,
        //                             Car_Destination.x - Car_Coordinate.x);
        // // double angle_diff = fabs(Normalize_Angle(angle_to_goal - Now_Angle));

        // Ok_List.push_back(
        //         {Dist_To_Obstacle, Dist_To_Goal, Velocity, Angular_Velocity, Velocity, Angular_Velocity, Dist_To_D_Star});
    // }

    // for (auto &i : Ok_List) {
    //     i.Dist_To_Obstacle =
    //         (i.Dist_To_Obstacle - MIN_Dist_To_Obstacle) / (MAX_Dist_To_Obstacle - MIN_Dist_To_Obstacle + 1e-8);
    //     i.Dist_To_Goal =
    //         (i.Dist_To_Goal - MIN_Dist_To_Goal) / (MAX_Dist_To_Goal - MIN_Dist_To_Goal + 1e-8);
    //     i.Velocity =
    //         (i.Velocity - MIN_Velocity) / (MAX_Velocity - MIN_Velocity + 1e-8);
    //     i.Angular_Velocity =
    //         (i.Angular_Velocity - MIN_Angular_Velocity) / (MAX_Angular_Velocity - MIN_Angular_Velocity + 1e-8);
    //     i.Dist_To_D_Star =
    //         (i.Dist_To_D_Star - MIN_Dist_To_D_Star) / (MAX_Dist_To_D_Star - MIN_Dist_To_D_Star + 1e-8);
    // }

    // double MAX_Score = -1e100;
    // Pair Target = {0, 0};

    // for (auto &i : Ok_List) {
    //     double Now_Score = 0;
    //     Now_Score += Beta * i.Dist_To_Goal;
    //     Now_Score += Gamma * i.Velocity;
    //     // Now_Score += Alpha * i.Dist_To_Obstacle;
    //     Now_Score += Delta2 * i.Dist_To_D_Star;

    //     if (Now_Score > MAX_Score) {
    //         MAX_Score = Now_Score;
    //         Target.Target_Angular_Velocity = i.ANGULAR_VELOCITY;
    //         Target.Target_Velocity = i.VELOCITY;
    //     }
    // }
    // return Target;
    }
}
