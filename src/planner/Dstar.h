/* Dstar.h
 * James Neufeld (neufeld@cs.ualberta.ca)
 * Compilation fixed by Arek Sredzki (arek@sredzki.com)
 */

#ifndef DSTAR_H
#define DSTAR_H

#include <iostream>
#include <thread>
#include <mutex>
#include <math.h>
#include <stack>
#include <queue>
#include <list>
#include <stdio.h>
#include <ext/hash_map>
#include <yaml-cpp/yaml.h>
#include <unsupported/Eigen/Splines>

#include "../models/state.h"
#include "../models/robot.h"

using namespace std;
using namespace __gnu_cxx;


struct ipoint2 {
    int x,y;
};

struct cellInfo {

    double g;
    double rhs;
    double cost;

};

class state_hash {
public:
    size_t operator()(const state &s) const {
        return s.x + 34245*s.y;
    }
};


typedef priority_queue<state, vector<state>, greater<state> > ds_pq;
typedef hash_map<state,cellInfo, state_hash, equal_to<state> > ds_ch;
typedef hash_map<state, float, state_hash, equal_to<state> > ds_oh;


class Dstar {

public:

    Dstar(const YAML::Node& config);
    ~Dstar();
    void   init(float sX, float sY, float gX, float gY);
    void   updateCell(float x, float y, double val);
    void   updateStart(float x, float y);
    void   updateGoal(float x, float y);
    bool   replan(int id);
    void   addCircularObstacle(float cx, float cy, float outerRadius, float innerRadius);
    void   addFieldObstacle();
    void   resetMap();
    void   start();
    void   stop();
    void   update(Robot* ourRobots, Robot* enemyRobots);
    void   run();

    array<vector<Eigen::Vector2d>, 16> getPlans();
    Eigen::Spline2d generateSpline(const std::vector<Eigen::Vector2d>& points);

private:

    const YAML::Node& conf;

    list<state> plans[16];
    list<state> tmpPlans[16];

    double C1;
    double k_m;
    state s_start, s_goal, s_last;
    int maxSteps;

    ds_pq openList;
    ds_ch cellHash;
    ds_oh openHash;

    float MAX_V;     
    float MAX_W;    
    float DT;        
    float PREDICT_TIME;
    float GOAL_TOLERANCE;

    bool   close(double x, double y);
    void   makeNewCell(state u);
    double getG(state u);
    double getRHS(state u);
    void   setG(state u, double g);
    double setRHS(state u, double rhs);
    double eightCondist(state a, state b);
    int    computeShortestPath();
    void   updateVertex(state u);
    void   insert(state u);
    void   remove(state u);
    double trueDist(state a, state b);
    double heuristic(state a, state b);
    state  calculateKey(state u);
    void   getSucc(state u, list<state> &s);
    void   getPred(state u, list<state> &s);
    double cost(state a, state b);
    bool   occupied(state u);
    bool   isValid(state u);
    float  keyHashCode(state u);

    float dRatio;

    std::chrono::high_resolution_clock::time_point beforeTime;

    bool running_;
    std::thread dstarThread_;
    std::mutex plansMutex;

    Robot ourRobots[16];
    Robot enemyRobots[16];

    vector<int> ourIDs;
    vector<int> enemyIDs;
};

struct RobotState {
    float x, y, theta;
    float v, w;       
};

struct VelocityCommand {
    float v;
    float w;
};



#endif
