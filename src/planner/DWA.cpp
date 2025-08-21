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
    return true;
    // std::cout<< "Checking coordinate: " << x.x << ", " << x.y << std::endl;
    // if (x.x < 0 || x.x >= Max_Range || x.y < 0 || x.y >= Max_Range) return false;
    // else return true;
}

bool DWA::Get_Trajectory(Robot bot) {
    Eigen::Vector2d position;
    double Time_Sum = 0;
    while (Time_Sum <= predictDelta) {
        Time_Sum += delta;
        double Next_Angle = bot.orientation + bot.angularVelocity * delta;
        // position.x() += bot.velocity * cos(Next_Angle) * delta;
        // position.y() += bot.velocity * sin(Next_Angle) * delta;
        bot.orientation = Next_Angle;
        if (Legal_Coordinate(position)) {
            Trajectory.push_back(position);
        } else {
            return false;
        }
    }
    if (Trajectory.empty()) return false;
    return true;
}

void DWA::update(Robot* ourRobots, Robot* enemyRobots, vector<Eigen::Vector2d>* dstarPlans) {
    for (int i = 0; i < conf["General"]["MaxRobotCount"].as<int>(); ++i) {
        this->ourRobots[i] = ourRobots[i];
        this->enemyRobots[i] = enemyRobots[i];
        if (dstarPlans) {
            this->dstarPlans[i] = dstarPlans[i];
        }
    }
}

RobotCmd* DWA::getDwa() {
    std::lock_guard<std::mutex> lock(dwaMutex);
    return this->robotCmds;
}

void DWA::run() {
    while (running_) {
        Refresh_Programme();

        for (int i = 0; i < 11; ++i) {
            if (enemyRobots[i].active) {
                Obstacle_Set.push_back({enemyRobots[i].x, enemyRobots[i].y});
            }
        }
        
        for (int i = 0; i < maxRobotCount; i++) {
            trajectory(dstarPlans[i], ourRobots[i]);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void DWA::trajectory(vector<Eigen::Vector2d> dstarPlan, Robot bot) {
    for (double Velocity = bot.velocity - predictDelta * maxVelocityAcc; Velocity <= bot.velocity + predictDelta * maxVelocityAcc; Velocity += velocityAccuracy) {
        if (fabs(Velocity) > maxVelocity) continue;

        Trajectory.clear();
        if (!Get_Trajectory(bot)) continue;

        double Break_Length = ((Velocity * Velocity / (2 * maxVelocityAcc)) / One_Block);
        double Dist_To_Obstacle = Get_Dist_To_Obstacle();
        // if (Dist_To_Obstacle != -1 && Dist_To_Obstacle < Safe_Distance) continue;
        // if (Dist_To_Obstacle != -1 && Break_Length > Dist_To_Obstacle) continue;

        double Dist_To_Goal = Get_Dist_To_Goal(bot.dest);
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
        Now_Score += Goal * i.Dist_To_Goal;
        Now_Score += Velocity * i.Velocity;
        // Now_Score += Alpha * i.Dist_To_Obstacle;
        Now_Score += D_Star * i.Dist_To_D_Star;

        if (Now_Score > MAX_Score) {
            MAX_Score = Now_Score;
            robotCmds[bot.robotId].id = bot.robotId;
            robotCmds[bot.robotId].vel = Eigen::Vector2d(
                i.Velocity * cos(bot.orientation),
                i.Velocity * sin(bot.orientation));
        }
    }
}
