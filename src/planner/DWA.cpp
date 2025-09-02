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

    Goal = conf["Planner"]["Goal"].as<float>();
    Velocity = conf["Planner"]["Velocity"].as<float>();
    D_Star = conf["Planner"]["D_Star"].as<float>();

    for (int i = 0; i < maxRobotCount; ++i) {
        robotCmds[i] = RobotCmd();
    }
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

bool Legal_Coordinate(Eigen::Vector2d pos) {
    // if (pos.x() > 0) {
    //     return false;
    // }
    return true;
}

bool DWA::Get_Trajectory(Robot bot, double Velocity, vector<Eigen::Vector2d> dstarPlan) {
    Eigen::Vector2d position = bot.pos;
    double Time_Sum = 0;
    while (Time_Sum <= predictDelta) {
        Time_Sum += delta;
        double botDestRad = atan2(bot.dest.y() - bot.pos.y(), bot.dest.x() - bot.pos.x());
        if (!dstarPlan.empty() && dstarPlan[0].allFinite()) {
            botDestRad = atan2(dstarPlan[0].y() - bot.pos.y(), dstarPlan[0].x() - bot.pos.x());
                // std::cout << "size: " << dstarPlan.size() << std::endl;
                // std::cout << "D_star[0]: " << dstarPlan[0].x() << " " << dstarPlan[0].y() << std::endl;
        }
        double Next_Angle = bot.orientation + bot.angularVelocity * delta;
        position.x() += Velocity * cos(botDestRad) * delta;
        position.y() += Velocity * sin(botDestRad) * delta;
        if (Legal_Coordinate(position)) {
            Trajectory.push_back(position);
        } else {
            return false;
        }
    }
    if (Trajectory.empty()) return false;
    return true;
}

void DWA::update(Robot* ourRobots, Robot* enemyRobots, array<vector<Eigen::Vector2d>, 16>& dstarPlans) {
    for (int i = 0; i < conf["General"]["MaxRobotCount"].as<int>(); ++i) {
        this->ourRobots[i] = ourRobots[i];
        this->enemyRobots[i] = enemyRobots[i];
        if (!dstarPlans[i].empty()) {
            // this->dstarPlans[i].clear();
            this->dstarPlans[i] = dstarPlans[i];
            // for (const auto& point : dstarPlans[i]) {
            //     this->dstarPlans[i].push_back(Eigen::Vector2d(point.x(), point.y()));
            // }
        }
    }
}

RobotCmd* DWA::getDwa() {
    std::lock_guard<std::mutex> lock(dwaMutex);
    return this->robotCmds;
}

void DWA::run() {
    while (running_) {
        Obstacle_Set.clear();

        for (int i = 0; i < 11; ++i) {
            if (enemyRobots[i].active) {
                Obstacle_Set.push_back({enemyRobots[i].pos.x(), enemyRobots[i].pos.y()});
            }
        }

        for (int i = 0; i < maxRobotCount; i++) {
            Refresh_Programme();
            trajectory(dstarPlans[i], ourRobots[i]);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void DWA::trajectory(vector<Eigen::Vector2d> dstarPlan, Robot bot) {
    for (double Velocity = bot.velocity - predictDelta * maxVelocityAcc; Velocity <= bot.velocity + predictDelta * maxVelocityAcc; Velocity += velocityAccuracy) {
        // std::cout << Velocity << std::endl;
        if (fabs(Velocity) > maxVelocity) continue;

        Trajectory.clear();
        if (!Get_Trajectory(bot, Velocity, dstarPlan)) continue;

        double Break_Length = ((Velocity * Velocity / (2 * maxVelocityAcc)) / One_Block);
        // double Dist_To_Obstacle = Get_Dist_To_Obstacle();
        double Dist_To_Obstacle = 0;
        // if (Dist_To_Obstacle != -1 && Dist_To_Obstacle < Safe_Distance) continue;
        // if (Dist_To_Obstacle != -1 && Break_Length > Dist_To_Obstacle) continue;

        double Dist_To_Goal = Get_Dist_To_Goal(bot.dest);
        if (Break_Length > Dist_To_Goal) {
            continue;
        }
        double Dist_To_D_Star = Get_D_Star_Dist(dstarPlan, Trajectory[Trajectory.size() - 1]);

        MIN_Dist_To_Obstacle = MIN(Dist_To_Obstacle, MIN_Dist_To_Obstacle);
        MAX_Dist_To_Obstacle = MAX(Dist_To_Obstacle, MAX_Dist_To_Obstacle);
        MIN_Dist_To_Goal = MIN(Dist_To_Goal, MIN_Dist_To_Goal);
        MAX_Dist_To_Goal = MAX(Dist_To_Goal, MAX_Dist_To_Goal);
        MIN_Velocity = MIN(Velocity, MIN_Velocity);
        MAX_Velocity = MAX(Velocity, MAX_Velocity);
        MIN_Dist_To_D_Star = MIN(Dist_To_D_Star, MIN_Dist_To_D_Star);
        MAX_Dist_To_D_Star = MAX(Dist_To_D_Star, MAX_Dist_To_D_Star);

        double angle_to_goal = atan2(bot.dest.y() - bot.pos.y(),
                                      bot.dest.x() - bot.pos.x());
        // // double angle_diff = fabs(Normalize_Angle(angle_to_goal - Now_Angle));
        
        Ok_List.push_back(
                {Dist_To_Obstacle, Dist_To_Goal, Velocity, Velocity, Dist_To_D_Star});
    }

    for (auto &i : Ok_List) {
        i.Dist_To_Obstacle =
            (i.Dist_To_Obstacle - MIN_Dist_To_Obstacle) / (MAX_Dist_To_Obstacle - MIN_Dist_To_Obstacle + 1e-8);
        i.Dist_To_Goal =
            (i.Dist_To_Goal - MIN_Dist_To_Goal) / (MAX_Dist_To_Goal - MIN_Dist_To_Goal + 1e-8);
        i.Velocity =
            (i.Velocity - MIN_Velocity) / (MAX_Velocity - MIN_Velocity + 1e-8);
        i.Dist_To_D_Star =
            (i.Dist_To_D_Star - MIN_Dist_To_D_Star) / (MAX_Dist_To_D_Star - MIN_Dist_To_D_Star + 1e-8);
    }
    double MAX_Score = -1e100;
    for (auto &i : Ok_List) {
        double Now_Score = 0;
        Now_Score += Goal * (1.0 - i.Dist_To_Goal);
        Now_Score += Velocity * i.Velocity;
        // Now_Score += Alpha * i.Dist_To_Obstacle;
        Now_Score += D_Star * (1.0 - i.Dist_To_D_Star);

        if (Now_Score > MAX_Score) {
            MAX_Score = Now_Score;
            robotCmds[bot.robotId].id = bot.robotId;

            double botDestRad;
            botDestRad = atan2(bot.dest.y() - bot.pos.y(), bot.dest.x() - bot.pos.x());
            if (!dstarPlan.empty() && dstarPlan[0].allFinite()) {
                botDestRad = atan2(dstarPlan[15].y() - bot.pos.y(), dstarPlan[15].x() - bot.pos.x());
                std::cout << "Bot_Pos: " << bot.pos.x() << " " << bot.pos.y() << std::endl;
                std::cout << "D_star[0]: " << dstarPlan[15].x() << " " << dstarPlan[15].y() << std::endl;

                std::cout << "Rad: " << botDestRad*180/3.14 << std::endl;
                std::cout << "botRad: " << atan2(bot.dest.y() - bot.pos.y(), bot.dest.x() - bot.pos.x())*180/3.14 << std::endl;
                std::cout << " " << std::endl;

            }


            double vx = i.VELOCITY * cos(botDestRad);
            double vy = i.VELOCITY * sin(botDestRad);

            double theta = bot.orientation;

            double veltangent = cos(theta) * vx + sin(theta) * vy;
            double velnormal = -sin(theta) * vx + cos(theta) * vy;

            robotCmds[bot.robotId].vel = Eigen::Vector2d(veltangent, velnormal);
        }
    }
    // std::cout << "robotPos: " << bot.pos.x() << " " << bot.pos.y() << std::endl;
    // std::cout << "velocity: " << bot.velocity << std::endl;
    // std::cout << "Selected velocity: " << sqrt(robotCmds[bot.robotId].vel.x() * robotCmds[bot.robotId].vel.x() + robotCmds[bot.robotId].vel.y() * robotCmds[bot.robotId].vel.y()) << std::endl;
}
