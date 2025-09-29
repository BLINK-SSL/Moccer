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
    int enemyID;

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
    void   updateCell(float x, float y, double val);
    void   updateStart(const Robot robot);
    state  findReachableGoal(state originalGoal);
    void   updateGoal(const Robot robot);
    bool   replan(int id, Robot robot);
    void   addEnemyObstacle(const Robot enemy);
    void   addFieldObstacle(int id);
    void   resetMap();
    void   start();
    void   stop();
    

    vector<Eigen::Vector2d>   run(const std::vector<Robot>& ourRobots, const std::vector<Robot>& enemyRobots, int id);

    Eigen::Spline2d generateSpline(const std::vector<Eigen::Vector2d>& points);

private:

    const YAML::Node& conf;

    vector<state> plans;

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
    int    computeShortestPath(int id);
    void   updateVertex(state u);
    void   insert(state u);
    void   remove(state u);
    double trueDist(state a, state b);
    double heuristic(state a, state bm);
    state  calculateKey(state u);
    void   getSucc(state u, list<state> &s);
    void   getPred(state u, list<state> &s);
    double cost(state a, state b);
    bool   occupied(state u);
    bool   isValid(state u);
    float  keyHashCode(state u);

    float dRatio;

    std::chrono::high_resolution_clock::time_point beforeTime;

    Robot ourRobots[16];
    Robot enemyRobots[16];

    vector<int> ourIDs;
    vector<int> enemyIDs;

    int id;
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
