/* Dstar.cpp
 * James Neufeld (neufeld@cs.ualberta.ca)
 * Compilation fixed by Arek Sredzki (arek@sredzki.com)
 */

#include "Dstar.h"


#include <iostream>
#include <chrono>
using namespace std::chrono;
/* void Dstar::Dstar()
 * --------------------------
 * Constructor sets constants.
 */
Dstar::Dstar() {
    maxSteps = 80000;  // node expansions before we give up
    C1       = 1;      // cost of an unseen cell
}

Dstar::~Dstar() {
    stop();
}

void Dstar::start() {
    running_ = true;  
    dstarThread_ = std::thread(&Dstar::run, this);
}

void Dstar::stop() {
    running_ = false;
    if (dstarThread_.joinable()) {
        dstarThread_.join();
    }
}

/* float Dstar::keyHashCode(state u)
 * --------------------------
 * Returns the key hash code for the state u, this is used to compare
 * a state that have been updated
 */
float Dstar::keyHashCode(state u) {

  return (float)(u.k.first + 1193*u.k.second);

}

/* bool Dstar::isValid(state u)
 * --------------------------
 * Returns true if state u is on the open list or not by checking if
 * it is in the hash table.
 */
bool Dstar::isValid(state u) {

  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return false;
  if (!close(keyHashCode(u), cur->second)) return false;
  return true;

}

/* void Dstar::getPath()
 * --------------------------
 * Returns the path created by replan()
 */
list<state> Dstar::getPath() {
  return path;
}

/* bool Dstar::occupied(state u)
 * --------------------------
 * returns true if the cell is occupied (non-traversable), false
 * otherwise. non-traversable are marked with a cost < 0.
 */
bool Dstar::occupied(state u) {

  ds_ch::iterator cur = cellHash.find(u);

  if (cur == cellHash.end()) return false;
  return (cur->second.cost < 0);
}

/* void Dstar::init(int sX, int sY, int gX, int gY)
 * --------------------------
 * Init dstar with start and goal coordinates, rest is as per
 * [S. Koenig, 2002]
 */
void Dstar::init(int sX, int sY, int gX, int gY, int dRatio) {
  this->dRatio = dRatio;
  cellHash.clear();
  path.clear();
  openHash.clear();
  while(!openList.empty()) openList.pop();

  k_m = 0;

  s_start.x = sX;
  s_start.y = sY;
  s_goal.x  = gX;
  s_goal.y  = gY;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;

}
/* void Dstar::makeNewCell(state u)
 * --------------------------
 * Checks if a cell is in the hash table, if not it adds it in.
 */
void Dstar::makeNewCell(state u) {

  if (cellHash.find(u) != cellHash.end()) return;

  cellInfo tmp;
  tmp.g       = tmp.rhs = heuristic(u,s_goal);
  tmp.cost    = C1;
  cellHash[u] = tmp;

}

/* double Dstar::getG(state u)
 * --------------------------
 * Returns the G value for state u.
 */
double Dstar::getG(state u) {

  if (cellHash.find(u) == cellHash.end())
    return heuristic(u,s_goal);
  return cellHash[u].g;

}

/* double Dstar::getRHS(state u)
 * --------------------------
 * Returns the rhs value for state u.
 */
double Dstar::getRHS(state u) {

  if (u == s_goal) return 0;

  if (cellHash.find(u) == cellHash.end())
    return heuristic(u,s_goal);
  return cellHash[u].rhs;

}

/* void Dstar::setG(state u, double g)
 * --------------------------
 * Sets the G value for state u
 */
void Dstar::setG(state u, double g) {

  makeNewCell(u);
  cellHash[u].g = g;
}

/* void Dstar::setRHS(state u, double rhs)
 * --------------------------
 * Sets the rhs value for state u
 */
double Dstar::setRHS(state u, double rhs) {

  makeNewCell(u);
  cellHash[u].rhs = rhs;

}

/* double Dstar::eightCondist(state a, state b)
 * --------------------------
 * Returns the 8-way distance between state a and state b.
 */
double Dstar::eightCondist(state a, state b) {
  double temp;
  double min = fabs(a.x - b.x);
  double max = fabs(a.y - b.y);
  if (min > max) {
    double temp = min;
    min = max;
    max = temp;
  }
  return ((M_SQRT2-1.0)*min + max);
}

/* int Dstar::computeShortestPath()
 * --------------------------
 * As per [S. Koenig, 2002] except for 2 main modifications:
 * 1. We stop planning after a number of steps, 'maxsteps' we do this
 *    because this algorithm can plan forever if the start is
 *    surrounded by obstacles.
 * 2. We lazily remove states from the open list so we never have to
 *    iterate through it.
 */
int Dstar::computeShortestPath() {

  list<state> s;
  list<state>::iterator i;

  if (openList.empty()) return 1;

  int k=0;
  while ((!openList.empty()) &&
         (openList.top() < (s_start = calculateKey(s_start))) ||
         (getRHS(s_start) != getG(s_start))) {

    if (k++ > maxSteps) {
      fprintf(stderr, "At maxsteps\n");
      return -1;
    }


    state u;

    bool test = (getRHS(s_start) != getG(s_start));

    // lazy remove
    while(1) {
      if (openList.empty()) return 1;
      u = openList.top();
      openList.pop();

      if (!isValid(u)) continue;
      if (!(u < s_start) && (!test)) return 2;
      break;
    }

    ds_oh::iterator cur = openHash.find(u);
    openHash.erase(cur);

    state k_old = u;

    if (k_old < calculateKey(u)) { // u is out of date
      insert(u);
    } else if (getG(u) > getRHS(u)) { // needs update (got better)
      setG(u,getRHS(u));
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
    } else {   // g <= rhs, state has got worse
      setG(u,INFINITY);
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
      updateVertex(u);
    }
  }
  return 0;
}

/* bool Dstar::close(double x, double y)
 * --------------------------
 * Returns true if x and y are within 10E-5, false otherwise
 */
bool Dstar::close(double x, double y) {

  if (isinf(x) && isinf(y)) return true;
  return (fabs(x-y) < 0.00001);

}

/* void Dstar::updateVertex(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void Dstar::updateVertex(state u) {

  list<state> s;
  list<state>::iterator i;

  if (u != s_goal) {
    getSucc(u,s);
    double tmp = INFINITY;
    double tmp2;

    for (i=s.begin();i != s.end(); i++) {
      tmp2 = getG(*i) + cost(u,*i);
      if (tmp2 < tmp) tmp = tmp2;
    }
    if (!close(getRHS(u),tmp)) setRHS(u,tmp);
  }

  if (!close(getG(u),getRHS(u))) insert(u);

}

/* void Dstar::insert(state u)
 * --------------------------
 * Inserts state u into openList and openHash.
 */
void Dstar::insert(state u) {

  ds_oh::iterator cur;
  float csum;

  u    = calculateKey(u);
  cur  = openHash.find(u);
  csum = keyHashCode(u);
  // return if cell is already in list. TODO: this should be
  // uncommented except it introduces a bug, I suspect that there is a
  // bug somewhere else and having duplicates in the openList queue
  // hides the problem...
  //if ((cur != openHash.end()) && (close(csum,cur->second))) return;

  openHash[u] = csum;
  openList.push(u);
}

/* void Dstar::remove(state u)
 * --------------------------
 * Removes state u from openHash. The state is removed from the
 * openList lazilily (in replan) to save computation.
 */
void Dstar::remove(state u) {

  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return;
  openHash.erase(cur);
}


/* double Dstar::trueDist(state a, state b)
 * --------------------------
 * Euclidean cost between state a and state b.
 */
double Dstar::trueDist(state a, state b) {

  float x = a.x-b.x;
  float y = a.y-b.y;
  return sqrt(x*x + y*y);

}

/* double Dstar::heuristic(state a, state b)
 * --------------------------
 * Pretty self explanitory, the heristic we use is the 8-way distance
 * scaled by a constant C1 (should be set to <= min cost).
 */
double Dstar::heuristic(state a, state b) {
  return eightCondist(a,b)*C1;
}

/* state Dstar::calculateKey(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
state Dstar::calculateKey(state u) {

  double val = fmin(getRHS(u),getG(u));

  u.k.first  = val + heuristic(u,s_start) + k_m;
  u.k.second = val;

  return u;

}

/* double Dstar::cost(state a, state b)
 * --------------------------
 * Returns the cost of moving from state a to state b. This could be
 * either the cost of moving off state a or onto state b, we went with
 * the former. This is also the 8-way cost.
 */
double Dstar::cost(state a, state b) {

  int xd = fabs(a.x-b.x);
  int yd = fabs(a.y-b.y);
  double scale = 1;

  if (xd+yd>1) scale = M_SQRT2;

  if (cellHash.count(a) == 0) return scale*C1;
  return scale*cellHash[a].cost;

}
/* void Dstar::updateCell(int x, int y, double val)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void Dstar::updateCell(int x, int y, double val) {

  state u;

  u.x = x;
  u.y = y;

  if ((u == s_start) || (u == s_goal)) return;

  makeNewCell(u);

  cellHash[u].cost = val;
  updateVertex(u);
}

/* void Dstar::getSucc(state u,list<state> &s)
 * --------------------------
 * Returns a list of successor states for state u, since this is an
 * 8-way graph this list contains all of a cells neighbours. Unless
 * the cell is occupied in which case it has no successors.
 */
// void Dstar::getSucc(state u,list<state> &s) {

//   s.clear();
//   u.k.first  = -1;
//   u.k.second = -1;

//   if (occupied(u)) return;

//   u.x += 1;
//   s.push_front(u);
//   u.y += 1;
//   s.push_front(u);
//   u.x -= 1;
//   s.push_front(u);
//   u.x -= 1;
//   s.push_front(u);
//   u.y -= 1;
//   s.push_front(u);
//   u.y -= 1;
//   s.push_front(u);
//   u.x += 1;
//   s.push_front(u);
//   u.x += 1;
//   s.push_front(u);

// }
void Dstar::getSucc(state u,list<state> &s) {
  s.clear();

  // 4近傍（上下左右）はそのまま追加
  const int dx4[4] = { 1, 0, -1, 0 };
  const int dy4[4] = { 0, 1,  0, -1 };
  for (int i = 0; i < 4; ++i) {
    state v = u;
    v.x += dx4[i];
    v.y += dy4[i];
    if (!occupied(v)) s.push_back(v);
  }

  // 斜め4方向：両方の隣接セルもチェック
  const int dx8[4] = {  1, -1, -1, 1 };
  const int dy8[4] = {  1,  1, -1,-1 };
  for (int i = 0; i < 4; ++i) {
    state v = u;
    v.x += dx8[i];
    v.y += dy8[i];

    // チェックする 2 つの直交セル
    state side1 = u;
    side1.x += dx8[i];
    side1.y += 0;
    state side2 = u;
    side2.x += 0;
    side2.y += dy8[i];

    if (!occupied(v) && !occupied(side1) && !occupied(side2)) {
      s.push_back(v);
    }
  }
}
/* void Dstar::getPred(state u,list<state> &s)
 * --------------------------
 * Returns a list of all the predecessor states for state u. Since
 * this is for an 8-way connected graph the list contails all the
 * neighbours for state u. Occupied neighbours are not added to the
 * list.
 */
void Dstar::getPred(state u,list<state> &s) {

  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.y += 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);

}

/* void Dstar::updateStart(int x, int y)
 * --------------------------
 * Update the position of the robot, this does not force a replan.
 */
void Dstar::updateStart(int x, int y) {
  s_start.x = x;
  s_start.y = y;

  k_m += heuristic(s_last,s_start);

  s_start = calculateKey(s_start);
  s_last  = s_start;

}

/* void Dstar::updateGoal(int x, int y)
 * --------------------------
 * This is somewhat of a hack, to change the position of the goal we
 * first save all of the non-empty on the map, clear the map, move the
 * goal, and re-add all of non-empty cells. Since most of these cells
 * are not between the start and goal this does not seem to hurt
 * performance too much. Also it free's up a good deal of memory we
 * likely no longer use.
 */
void Dstar::updateGoal(int x, int y) {

  list< pair<ipoint2, double> > toAdd;
  pair<ipoint2, double> tp;

  ds_ch::iterator i;
  list< pair<ipoint2, double> >::iterator kk;

  for(i=cellHash.begin(); i!=cellHash.end(); i++) {
    if (!close(i->second.cost, C1)) {
      tp.first.x = i->first.x;
      tp.first.y = i->first.y;
      tp.second = i->second.cost;
      toAdd.push_back(tp);
    }
  }

  cellHash.clear();
  openHash.clear();

  while(!openList.empty())
    openList.pop();

  k_m = 0;

  s_goal.x  = x;
  s_goal.y  = y;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;

  for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
    updateCell(kk->first.x, kk->first.y, kk->second);
  }


}

/* bool Dstar::replan()
 * --------------------------
 * Updates the costs for all cells and computes the shortest path to
 * goal. Returns true if a path is found, false otherwise. The path is
 * computed by doing a greedy search over the cost+g values in each
 * cells. In order to get around the problem of the robot taking a
 * path that is near a 45 degree angle to goal we break ties based on
 *  the metric euclidean(state, goal) + euclidean(state,start).
 */
bool Dstar::replan() {

  path.clear();

  int res = computeShortestPath();
  //printf("res: %d ols: %d ohs: %d tk: [%f %f] sk: [%f %f] sgr: (%f,%f)\n",res,openList.size(),openHash.size(),openList.top().k.first,openList.top().k.second, s_start.k.first, s_start.k.second,getRHS(s_start),getG(s_start));
  if (res < 0) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }
  list<state> n;
  list<state>::iterator i;

  state cur = s_start;

  if (isinf(getG(s_start))) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }

  while(cur != s_goal) {

    path.push_back(cur);
    getSucc(cur, n);

    if (n.empty()) {
      fprintf(stderr, "NO PATH TO GOAL\n");
      return false;
    }

    double cmin = INFINITY;
    double tmin;
    state smin;

    for (i=n.begin(); i!=n.end(); i++) {

      //if (occupied(*i)) continue;
      double val  = cost(cur,*i);
      double val2 = trueDist(*i,s_goal) + trueDist(s_start,*i); // (Euclidean) cost to goal + cost to pred
      val += getG(*i);

      if (close(val,cmin)) {
        if (tmin > val2) {
          tmin = val2;
          cmin = val;
          smin = *i;
        }
      } else if (val < cmin) {
        tmin = val2;
        cmin = val;
        smin = *i;
      }
    }
    n.clear();
    cur = smin;
  }
  path.push_back(s_goal);
  return true;
}

Pair Dstar::draw(float dRatio, Robot* blueRobots, Robot* yellowRobots) {

  ds_ch::iterator iter;
  ds_oh::iterator iter1;
  state t;

  list<state>::iterator iter2;

  glBegin(GL_QUADS);
  

  for(iter=cellHash.begin(); iter != cellHash.end(); iter++) {
    if (iter->second.cost == 1) glColor3f(0,1,0);
    else if (iter->second.cost < 0 ) glColor3f(1,0,0);
    else glColor3f(0,0,1);
    drawCell(iter->first,0.45);
  }

  glColor3f(1,1,0);
  drawCell(s_start,0.45);
  glColor3f(1,0,1);
  drawCell(s_goal,0.45);

  for(iter1=openHash.begin(); iter1 != openHash.end(); iter1++) {
    glColor3f(0.4,0,0.8);
    drawCell(iter1->first, 0.2);
  }


  glEnd();

  glLineWidth(4);
  glBegin(GL_LINE_STRIP);
  glColor3f(0.6, 0.1, 0.4);

  for(iter2=path.begin(); iter2 != path.end(); iter2++) {
    glVertex3f(iter2->x, iter2->y, 0.2);
  }
  std::vector<Coordinate> D_Star_Road;
  for (auto &coord : path) {
    D_Star_Road.push_back({coord.x * dRatio, coord.y * dRatio});
  }
  Coordinate s_start_coord = {s_start.x * dRatio, s_start.y * dRatio};
  Coordinate s_goal_coord = {s_goal.x * dRatio, s_goal.y * dRatio};
  Bot_Model bot_model = {5000.0, 20, 4000.0, 50}; // 60 degrees in radians
  // std::cout << "orientation: " << blueRobots[0].orientation << std::endl;
  float velocity = sqrt(blueRobots[0].velocity.x * blueRobots[0].velocity.x + blueRobots[0].velocity.y * blueRobots[0].velocity.y);
  // std::cout << "velocity: " << velocity << std::endl;
  // std::cout << "angularVelocity: " << blueRobots[0].angularVelocity << std::endl;
  DWAPlanner dwaPlanner;
  Pair pair = {};
  // Pair pair = dwaPlanner.DWA(D_Star_Road, s_start_coord, blueRobots[0].orientation, velocity, blueRobots[0].angularVelocity, s_goal_coord, bot_model, blueRobots, yellowRobots);
  // calculate fps
  // std::cout << "DWA Planner: " << pair.Target_Velocity << ", " << pair.Target_Angular_Velocity << std::endl;
  // std::cout << "pre_x: " << blueRobots[0].pre_x << ", pre_y: " << blueRobots[0].pre_y << std::endl;
  // std::cout << "Robot Position: (" << blueRobots[0].x << ", " << blueRobots[0].y << ")" << std::endl;
  // std::cout << "velocity: " << blueRobots[0].velocity.x << ", " << blueRobots[0].velocity.y << std::endl;
  // std::cout << "Robot Velocity: " << velocity << ", Robot Angular Velocity: " << blueRobots[0].angularVelocity << std::endl;

  // glEnd();


  // auto currentTime = high_resolution_clock::now();
  // duration<float> elapsed = currentTime - beforeTime;
  // beforeTime = currentTime;
  // printf("Dstar::draw() took %f seconds\n", elapsed.count());

  return pair;
}


void Dstar::drawCell(state s, float size) {

  float x = s.x;
  float y = s.y;

  // std::cout << "Drawing cell at: " << x << ", " << y << std::endl;

  size = 0.5;
  glVertex2f(x - size, y - size);
  glVertex2f(x + size, y - size);
  glVertex2f(x + size, y + size);
  glVertex2f(x - size, y + size);


}


void Dstar::addCircularObstacle(int cx, int cy, int outerRadius, int innerRadius) {
    // for (int x = cx - outerRadius; x <= cx + outerRadius; ++x) {
    //     for (int y = cy - outerRadius; y <= cy + outerRadius; ++y) {
    //         int dx = abs(x - cx);
    //         int dy = abs(y - cy);
    //         int manhattanDist = dx + dy;
    //         if (manhattanDist <= outerRadius && manhattanDist >= innerRadius) {
    //             updateCell(x, y, -1);
    //         }
    //     }
    // }
    outerRadius /= dRatio;
    innerRadius /= dRatio;
    for (int x = cx - outerRadius; x <= cx + outerRadius; ++x) {
        for (int y = cy - outerRadius; y <= cy + outerRadius; ++y) {
            int dx = x - cx;
            int dy = y - cy;
            int distSq = dx * dx + dy * dy;
            if (distSq <= outerRadius * outerRadius && distSq >= innerRadius * innerRadius) {
                updateCell(x, y, -1);
            }
        }
    }
}

void Dstar::addFieldObstacle() {
    float dRatio = 75.0;
    for (int i = -(6700 / dRatio); i < (6700 / dRatio); i++) {
      updateCell(i, 5200 / dRatio, -1);
      updateCell(i, -5200 / dRatio, -1);
    }
    for (int i = -(5200 / dRatio); i < (5200 / dRatio); i++) {
      updateCell(6700 / dRatio, i, -1);
      updateCell(-6700 / dRatio, i, -1);
    }
}

void Dstar::resetMap()
{
  cellHash.clear();
  openHash.clear();
  while(!openList.empty()) openList.pop();
  path.clear();
  k_m = 0;
}

void Dstar::update(Robot* blueRobots, Robot* yellowRobots) {
    for (int i = 0; i < 16; ++i) {
        this->blueRobots[i] = blueRobots[i];
        this->yellowRobots[i] = yellowRobots[i];
    }
}

void Dstar::run()
{
    while (running_) {
        float dRatio = 75.0;
        resetMap();
        updateStart(static_cast<int>(blueRobots[0].x / dRatio), static_cast<int>(blueRobots[0].y / dRatio));
        for (int i = 0; i < 11; ++i) {
            if (blueRobots[i].confidence > 0.5) {
                if (i == 0) continue;
                addCircularObstacle(static_cast<int>(blueRobots[i].x / dRatio), static_cast<int>(blueRobots[i].y / dRatio), 200, 0);
            }
            if (yellowRobots[i].confidence > 0.5) {
                addCircularObstacle(static_cast<int>(yellowRobots[i].x / dRatio), static_cast<int>(yellowRobots[i].y / dRatio), 200, 0);
            }
        }
        addFieldObstacle();
        // if (b_autoreplan) replan();
        std::vector<Coordinate> D_Star_Road;
        for (auto &coord : path) {
          D_Star_Road.push_back({coord.x * dRatio, coord.y * dRatio});
        }
        Coordinate s_start_coord = {s_start.x * dRatio, s_start.y * dRatio};
        Coordinate s_goal_coord = {s_goal.x * dRatio, s_goal.y * dRatio};
        Bot_Model bot_model = {5000.0, 20, 4000.0, 50}; // 60 degrees in radians
        float velocity = sqrt(blueRobots[0].velocity.x * blueRobots[0].velocity.x + blueRobots[0].velocity.y * blueRobots[0].velocity.y);
        _pair = dwaPlanner.DWA(D_Star_Road, s_start_coord, blueRobots[0].orientation, velocity, blueRobots[0].angularVelocity, s_goal_coord, bot_model, blueRobots, yellowRobots);
        std::cout << "Robot velocity: " << velocity << ", Angular Velocity: " << blueRobots[0].angularVelocity << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

Pair Dstar::getPair() {
  return _pair;
}