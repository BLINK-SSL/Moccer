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

#ifdef MACOS
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "DWA.h"
#include "../models/robot.h"

using namespace std;
using namespace __gnu_cxx;

class state {
 public:
  int x;
  int y;
  pair<double,double> k;

  bool operator == (const state &s2) const {
    return ((x == s2.x) && (y == s2.y));
  }

  bool operator != (const state &s2) const {
    return ((x != s2.x) || (y != s2.y));
  }

  bool operator > (const state &s2) const {
    if (k.first-0.00001 > s2.k.first) return true;
    else if (k.first < s2.k.first-0.00001) return false;
    return k.second > s2.k.second;
  }

  bool operator <= (const state &s2) const {
    if (k.first < s2.k.first) return true;
    else if (k.first > s2.k.first) return false;
    return k.second < s2.k.second + 0.00001;
  }


  bool operator < (const state &s2) const {
    if (k.first + 0.000001 < s2.k.first) return true;
    else if (k.first - 0.000001 > s2.k.first) return false;
    return k.second < s2.k.second;
  }

};

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

  Dstar();
  ~Dstar();
  void   init(int sX, int sY, int gX, int gY, int dRatio);
  void   updateCell(int x, int y, double val);
  void   updateStart(int x, int y);
  void   updateGoal(int x, int y);
  bool   replan();
  Pair   draw(float dRatio, Robot* blueRobots, Robot* yellowRobots);
  void   drawCell(state s,float z);
  void   addCircularObstacle(int cx, int cy, int outerRadius, int innerRadius);
  void   addFieldObstacle();
  void   resetMap();
  void   start();
  void   stop();
  void   update(Robot* blueRobots, Robot* yellowRobots);
  void   run();
  Pair   getPair();

  list<state> getPath();

 private:

  list<state> path;

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

  DWAPlanner dwaPlanner;
  std::chrono::high_resolution_clock::time_point beforeTime;

  bool running_;
  std::thread dstarThread_;

  Robot blueRobots[16];
  Robot yellowRobots[16];
  Pair _pair;
};

// struct Point {
//     float x, y;
// };

struct RobotState {
    float x, y, theta; // 座標と向き?��ラジアン?�?
    float v, w;        // 線速度と角速度
};

struct VelocityCommand {
    float v; // 線速度
    float w; // 角速度
};



#endif
