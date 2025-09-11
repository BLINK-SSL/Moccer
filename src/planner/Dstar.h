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
    void   updateCell(float x, float y, double val, int id);
    void   updateStart(float x, float y, int id);
    void   updateGoal(float x, float y, int id);
    bool   replan(int id);
    void   addCircularObstacle(float cx, float cy, float outerRadius, float innerRadius, int id);
    void   addFieldObstacle(int id);
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

    vector<double> C1;
    vector<double> k_ms;
    // state s_start, s_goal, s_last;
    vector<state> s_start;
    vector<state> s_goal;
    vector<state> s_last;
    int maxSteps;

    vector <ds_pq> openLists = vector<ds_pq>(16);
    vector <ds_ch> cellHashs = vector<ds_ch>(16);
    vector <ds_oh> openHashs = vector<ds_oh>(16);

    float MAX_V;     
    float MAX_W;    
    float DT;        
    float PREDICT_TIME;
    float GOAL_TOLERANCE;

    bool   close(double x, double y);
    void   makeNewCell(state u, int id);
    double getG(state u, int id);
    double getRHS(state u, int id);
    void   setG(state u, double g, int id);
    double setRHS(state u, double rhs, int id);
    double eightCondist(state a, state b);
    int    computeShortestPath(int id);
    void   updateVertex(state u, int id);
    void   insert(state u, int id);
    void   remove(state u, int id);
    double trueDist(state a, state b);
    double heuristic(state a, state bm, int id);
    state  calculateKey(state u, int id);
    void   getSucc(state u, list<state> &s, int id);
    void   getPred(state u, list<state> &s, int id);
    double cost(state a, state b, int id);
    bool   occupied(state u, int id);
    bool   isValid(state u, int id);
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
