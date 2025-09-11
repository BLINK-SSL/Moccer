/* Dstar.cpp
 * James Neufeld (neufeld@cs.ualberta.ca)
 * Compilation fixed by Arek Sredzki (arek@sredzki.com)
 */

#include "Dstar.h"


#include <iostream>
#include <chrono>
using namespace std::chrono;

Dstar::Dstar(const YAML::Node& config) : conf(config), running_(false) {
    maxSteps = 800000;  // node expansions before we give up
    C1       = 1;      // cost of an unseen cell
    dRatio   = conf["Planner"]["dRatio"].as<float>();

    for (int i = 0; i < conf["General"]["MaxRobotCount"].as<int>(); ++i) {
        plans[i] = list<state>();
        plans[i].push_back({0, 0});
    }
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

float Dstar::keyHashCode(state u) {
    return (float)(u.k.first + 1193*u.k.second);
}

bool Dstar::isValid(state u) {
    ds_oh::iterator cur = openHash.find(u);
    if (cur == openHash.end()) return false;
    if (!close(keyHashCode(u), cur->second)) return false;
    return true;
}

bool Dstar::occupied(state u) {
    ds_ch::iterator cur = cellHash.find(u);
    if (cur == cellHash.end()) return false;
    return (cur->second.cost < 0);
}

void Dstar::makeNewCell(state u) {

    if (cellHash.find(u) != cellHash.end()) return;

    cellInfo tmp;
    tmp.g       = tmp.rhs = heuristic(u,s_goal);
    tmp.cost    = C1;
    cellHash[u] = tmp;

}

double Dstar::getG(state u) {

    if (cellHash.find(u) == cellHash.end())
        return heuristic(u,s_goal);
    return cellHash[u].g;

}

double Dstar::getRHS(state u) {

    if (u == s_goal) return 0;

    if (cellHash.find(u) == cellHash.end())
        return heuristic(u,s_goal);
    return cellHash[u].rhs;

}

void Dstar::setG(state u, double g) {

    makeNewCell(u);
    cellHash[u].g = g;
}

double Dstar::setRHS(state u, double rhs) {
    makeNewCell(u);
    cellHash[u].rhs = rhs;
}

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

bool Dstar::close(double x, double y) {

    if (isinf(x) && isinf(y)) return true;
    return (fabs(x-y) < 0.00001);

}

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

void Dstar::remove(state u) {

    ds_oh::iterator cur = openHash.find(u);
    if (cur == openHash.end()) return;
    openHash.erase(cur);
}


double Dstar::trueDist(state a, state b) {

    float x = a.x-b.x;
    float y = a.y-b.y;
    return sqrt(x*x + y*y);

}

double Dstar::heuristic(state a, state b) {
    return eightCondist(a,b)*C1;
}

state Dstar::calculateKey(state u) {

    double val = fmin(getRHS(u),getG(u));

    u.k.first  = val + heuristic(u,s_start) + k_m;
    u.k.second = val;

    return u;

}

double Dstar::cost(state a, state b) {

    int xd = fabs(a.x-b.x);
    int yd = fabs(a.y-b.y);
    double scale = 1;

    if (xd+yd>1) scale = M_SQRT2;

    if (cellHash.count(a) == 0) return scale*C1;
    return scale*cellHash[a].cost;

}

void Dstar::updateCell(float x, float y, double val) {

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

    const int dx4[4] = { 1, 0, -1, 0 };
    const int dy4[4] = { 0, 1,  0, -1 };
    for (int i = 0; i < 4; ++i) {
        state v = u;
        v.x += dx4[i];
        v.y += dy4[i];
        if (!occupied(v)) s.push_back(v);
    }

    const int dx8[4] = {  1, -1, -1, 1 };
    const int dy8[4] = {  1,  1, -1,-1 };
    for (int i = 0; i < 4; ++i) {
        state v = u;
        v.x += dx8[i];
        v.y += dy8[i];

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
void Dstar::updateStart(float x, float y) {
    state u;
    u.x = x / dRatio;
    u.y = y / dRatio;

    if (occupied(u)) {
        fprintf(stderr, "Start position is inside an obstacle\n");

        // 近傍探索して空いているセルを探す
        bool found = false;
        for (double i = 0.5; i < 5 && !found; i += 0.5) {
            for (int dx = -i; dx <= i && !found; ++dx) {
                for (int dy = -i; dy <= i && !found; ++dy) {
                    state v = u;
                    v.x += dx;
                    v.y += dy;
                    if (!occupied(v)) {
                        u = v;
                        found = true;
                    }
                }
            }
        }

        if (!found) {
            fprintf(stderr, "No valid start position near obstacle\n");
            return;
        }
    }

    s_start.x = u.x;
    s_start.y = u.y;
    k_m += heuristic(s_last, s_start);
    s_start = calculateKey(s_start);
    s_last = s_start;
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
void Dstar::updateGoal(float x, float y) {
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
bool Dstar::replan(int id) {

    plans[id].clear();

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
        plans[id].push_back(cur);
        getSucc(cur, n);

        if (n.empty()) {
            fprintf(stderr, "NO PATH TO GOAL\n");
            return false;
        }

        double cmin = INFINITY;
        double tmin;
        state smin;

        for (i=n.begin(); i!=n.end(); i++) {
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
    plans[id].push_back(s_goal);
    
    tmpPlans[id] = plans[id];
    
    return true;
}

void Dstar::addCircularObstacle(float cx, float cy, float outerRadius, float innerRadius) {
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
    cx /= dRatio;
    cy /= dRatio;
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
    for (int i = 0; i < 16; ++i) {
        plans[i].clear();
    }
    k_m = 0;
}

void Dstar::update(Robot* ourRobots, Robot* enemyRobots) {
    for (int i = 0; i < 16; ++i) {
        this->ourRobots[i] = ourRobots[i];
        this->enemyRobots[i] = enemyRobots[i];
    }
    
}

// array<vector<Eigen::Vector2d>, 16> Dstar::getPlans() {
//     std::lock_guard<std::mutex> lock(plansMutex);
//     array<vector<Eigen::Vector2d>, 16> newPlans;
//     for (int i = 0; i < conf["General"]["MaxRobotCount"].as<int>(); i++) {
//         for (auto &p : tmpPlans[i]) {
//             newPlans[i].push_back(Eigen::Vector2d(p.x*dRatio, p.y*dRatio));
//         }
//     }
//     return newPlans;
// }

Eigen::Spline2d Dstar::generateSpline(const std::vector<Eigen::Vector2d>& points) {
    Eigen::MatrixXd pts(2, points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        pts(0, i) = points[i].x();
        pts(1, i) = points[i].y();
    }
    Eigen::RowVectorXd t = Eigen::RowVectorXd::LinSpaced(points.size(), 0.0, 1.0);
    return Eigen::SplineFitting<Eigen::Spline2d>::Interpolate(pts, 3, t);
}

array<vector<Eigen::Vector2d>, 16> Dstar::getPlans() {
    std::lock_guard<std::mutex> lock(plansMutex);
    array<vector<Eigen::Vector2d>, 16> newPlans;
    for (int i = 0; i < 1; i++) {
        for (auto &p : tmpPlans[i]) {
            newPlans[i].push_back(Eigen::Vector2d(p.x*dRatio, p.y*dRatio));
        }
    }
    return newPlans;
}

void Dstar::run() {
    while (running_) {
        resetMap();
        // addFieldObstacle();
        // for (int i = 0; i < conf["General"]["MaxRobotCount"].as<int>(); i++) {
        for (int i = 0; i < 11; i++) {
            if (!enemyRobots[i].active) continue;
            addCircularObstacle(enemyRobots[i].pos.x(), enemyRobots[i].pos.y(), 360, 0);
        }

        for (int i = 0; i < conf["General"]["MaxRobotCount"].as<int>(); i++) {
            if (!ourRobots[i].active) continue;
            updateStart(ourRobots[i].pos.x(), ourRobots[i].pos.y());
            updateGoal(0, 0);
            replan(i);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}   
