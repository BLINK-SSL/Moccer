/* Dstar.cpp
 * James Neufeld (neufeld@cs.ualberta.ca)
 * Compilation fixed by Arek Sredzki (arek@sredzki.com)
 */

#include "Dstar.h"

#include <iostream>
#include <chrono>
using namespace std::chrono;

Dstar::Dstar(const YAML::Node& config) : conf(config), running_(false) {
    maxSteps = 80000;  // node expansions before we give up
    // C1       = 1;      // cost of an unseen cell
    dRatio   = conf["Planner"]["dRatio"].as<float>();

    for (int i = 0; i < conf["General"]["MaxRobotCount"].as<int>(); ++i) {
        plans[i] = list<state>();
        plans[i].push_back({0, 0});

        ourIDs.push_back(i);
        enemyIDs.push_back(i);
        C1.push_back(1);
        k_ms.push_back(1.0/dRatio);
        s_start.push_back({0, 0});
        s_goal.push_back({0, 0});
        s_last.push_back({0, 0});

        openHashs[i] = ds_oh();
        openLists[i] = ds_pq();
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

bool Dstar::isValid(state u, int id) {
    ds_oh::iterator cur = openHashs[id].find(u);
    if (cur == openHashs[id].end()) return false;
    if (!close(keyHashCode(u), cur->second)) return false;
    return true;
}

bool Dstar::occupied(state u, int id) {
    ds_ch::iterator cur = cellHash.find(u);
    if (cur == cellHash.end()) return false;
    return (cur->second.cost < 0);
}

void Dstar::makeNewCell(state u, int id) {

    if (cellHash.find(u) != cellHash.end()) return;

    cellInfo tmp;
    tmp.g       = tmp.rhs = heuristic(u,s_goal[id], id);
    tmp.cost    = C1[id];
    cellHash[u] = tmp;

}

double Dstar::getG(state u, int id) {

    if (cellHash.find(u) == cellHash.end())
        return heuristic(u,s_goal[id], id);
    return cellHash[u].g;

}

double Dstar::getRHS(state u, int id) {

    if (u == s_goal[id]) return 0;

    if (cellHash.find(u) == cellHash.end())
        return heuristic(u,s_goal[id], id);
    return cellHash[u].rhs;

}

void Dstar::setG(state u, double g, int id) {

    makeNewCell(u, id);
    cellHash[u].g = g;
}

double Dstar::setRHS(state u, double rhs, int id) {
    makeNewCell(u, id);
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

int Dstar::computeShortestPath(int id) {

    list<state> s;
    list<state>::iterator i;

    if (openLists[id].empty()) return 1;

    int k=0;
    while ((!openLists[id].empty()) &&
            (openLists[id].top() < (s_start[id] = calculateKey(s_start[id], id))) ||
            (getRHS(s_start[id], id) != getG(s_start[id], id))) {
        // std::cout << "Dstar computing" << std::endl;
        // if (getRHS(s_start[id], id) != getG(s_start[id], id))
        //     std::cout << "1" << std::endl;
        // if (openLists[id].top() < (s_start[id] = calculateKey(s_start[id], id)))
        //     std::cout << "2" << std::endl;
        // if (!openLists[id].empty())
        //     std::cout << "3" << std::endl;

        if (k++ > maxSteps) {
            fprintf(stderr, "At maxsteps\n");
            return -1;
        }


        state u;

        bool test = (getRHS(s_start[id], id) != getG(s_start[id], id));

        // lazy remove
        while(1) {
            if (openLists[id].empty()) return 1;
            u = openLists[id].top();
            openLists[id].pop();

            if (!isValid(u, id)) continue;
            if (!(u < s_start[id]) && (!test)) return 2;
            break;
        }

        ds_oh::iterator cur = openHashs[id].find(u);
        openHashs[id].erase(cur);

        state k_old = u;

        if (k_old < calculateKey(u, id)) { // u is out of date
            insert(u, id);
        } else if (getG(u, id) > getRHS(u, id)) { // needs update (got better)
            setG(u,getRHS(u, id), id);
            getPred(u,s, id);
            for (i=s.begin();i != s.end(); i++) {
                updateVertex(*i, id);
            }
        } else {   // g <= rhs, state has got worse
            setG(u,INFINITY, id);
            getPred(u,s, id);
            for (i=s.begin();i != s.end(); i++) {
                updateVertex(*i, id);
            }
            updateVertex(u, id);
        }
    }
    return 0;
}

bool Dstar::close(double x, double y) {

    if (isinf(x) && isinf(y)) return true;
    return (fabs(x-y) < 0.00001);

}

void Dstar::updateVertex(state u, int id) {

    list<state> s;
    list<state>::iterator i;

    if (u != s_goal[id]) {
        getSucc(u,s, id);
        double tmp = INFINITY;
        double tmp2;

        for (i=s.begin();i != s.end(); i++) {
            tmp2 = getG(*i, id) + cost(u,*i, id);
            if (tmp2 < tmp) tmp = tmp2;
        }
        if (!close(getRHS(u, id),tmp)) setRHS(u,tmp, id);
    }

    if (!close(getG(u, id),getRHS(u, id))) insert(u, id);

}

void Dstar::insert(state u, int id) {

    ds_oh::iterator cur;
    float csum;

    u    = calculateKey(u, id);
    cur  = openHashs[id].find(u);
    csum = keyHashCode(u);
    // return if cell is already in list. TODO: this should be
    // uncommented except it introduces a bug, I suspect that there is a
    // bug somewhere else and having duplicates in the openList queue
    // hides the problem...
    //if ((cur != openHash.end()) && (close(csum,cur->second))) return;

    openHashs[id][u] = csum;
    openLists[id].push(u);
}

void Dstar::remove(state u, int id) {

    ds_oh::iterator cur = openHashs[id].find(u);
    if (cur == openHashs[id].end()) return;
    openHashs[id].erase(cur);
}


double Dstar::trueDist(state a, state b) {

    float x = a.x-b.x;
    float y = a.y-b.y;
    return sqrt(x*x + y*y);

}

double Dstar::heuristic(state a, state b, int id) {
    return eightCondist(a,b)*C1[id];
}

state Dstar::calculateKey(state u, int id) {

    double val = fmin(getRHS(u, id),getG(u, id));

    u.k.first  = val + heuristic(u,s_start[id], id) + k_ms[id];
    u.k.second = val;

    return u;

}

double Dstar::cost(state a, state b, int id) {

    int xd = fabs(a.x-b.x);
    int yd = fabs(a.y-b.y);
    double scale = 1;

    if (xd+yd>1) scale = M_SQRT2;

    if (cellHash.count(a) == 0) return scale*C1[id];
    return scale*cellHash[a].cost;

}

void Dstar::updateCell(float x, float y, double val, int id) {

    state u;

    u.x = x;
    u.y = y;

    if ((u == s_start[id]) || (u == s_goal[id])) return;

    makeNewCell(u, id);

    cellHash[u].cost = val;
    updateVertex(u, id);
}

void Dstar::getSucc(state u,list<state> &s, int id) {
    s.clear();

    const int dx4[4] = { 1, 0, -1, 0 };
    const int dy4[4] = { 0, 1,  0, -1 };
    for (int i = 0; i < 4; ++i) {
        state v = u;
        v.x += dx4[i];
        v.y += dy4[i];
        if (!occupied(v, id)) s.push_back(v);
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

        if (!occupied(v, id) && !occupied(side1, id) && !occupied(side2, id)) {
            s.push_back(v);
        }
    }
}

void Dstar::getPred(state u,list<state> &s, int id) {

    s.clear();
    u.k.first  = -1;
    u.k.second = -1;

    u.x += 1;
    if (!occupied(u, id)) s.push_front(u);
    u.y += 1;
    if (!occupied(u, id)) s.push_front(u);
    u.x -= 1;
    if (!occupied(u, id)) s.push_front(u);
    u.x -= 1;
    if (!occupied(u, id)) s.push_front(u);
    u.y -= 1;
    if (!occupied(u, id)) s.push_front(u);
    u.y -= 1;
    if (!occupied(u, id)) s.push_front(u);
    u.x += 1;
    if (!occupied(u, id)) s.push_front(u);
    u.x += 1;
    if (!occupied(u, id)) s.push_front(u);

}

void Dstar::updateStart(const Robot robot) {
    int id = robot.robotId;
    state u;
    u.x = robot.pos.x() / dRatio;
    u.y = robot.pos.y() / dRatio;

    if (occupied(u, id)) {
        bool found = false;
        for (double i = 0.5; i < 5 && !found; i += 0.5) {
            for (int dx = -i; dx <= i && !found; ++dx) {
                for (int dy = -i; dy <= i && !found; ++dy) {
                    state v = u;
                    v.x += dx;
                    v.y += dy;
                    if (!occupied(v, id)) {
                        u = v;
                        found = true;
                    }
                }
            }
        }
    }

    s_start[id].x = u.x;
    s_start[id].y = u.y;
    k_ms[id] += heuristic(s_last[id], s_start[id], id);
    s_start[id] = calculateKey(s_start[id], id);
    s_last[id] = s_start[id];
}

void Dstar::updateGoal(const Robot robot) {
    int id = robot.robotId;
    state u;
    u.x = robot.dest.x() / dRatio;
    u.y = robot.dest.y() / dRatio;

    if (occupied(u, id)) {
        std::cout << "Goal occupied, searching for nearest free cell" << std::endl;
        bool found = false;
        for (double i = 0.5; i < 5 && !found; i += 0.5) {
            for (int dx = -i; dx <= i && !found; ++dx) {
                for (int dy = -i; dy <= i && !found; ++dy) {
                    state v = u;
                    v.x += dx;
                    v.y += dy;
                    if (!occupied(v, id)) {
                        u = v;
                        found = true;
                    }
                }
            }
        }
    }

    s_goal[id].x = u.x;
    s_goal[id].y = u.y;

    list< pair<ipoint2, double> > toAdd;
    pair<ipoint2, double> tp;

    ds_ch::iterator i;
    list< pair<ipoint2, double> >::iterator kk;

    for(i=cellHash.begin(); i!=cellHash.end(); i++) {
        if (!close(i->second.cost, C1[id])) {
            tp.first.x = i->first.x;
            tp.first.y = i->first.y;
            tp.second = i->second.cost;
            toAdd.push_back(tp);
        }
    }

    cellHash.clear();
    openHashs[id].clear();

    while(!openLists[id].empty())
        openLists[id].pop();

    k_ms[id] = 0;

    // s_goal[id].x  = u.x;
    // s_goal[id].y  = u.y;

    cellInfo tmp;
    tmp.g = tmp.rhs =  0;
    tmp.cost = C1[id];

    cellHash[s_goal[id]] = tmp;

    tmp.g = tmp.rhs = heuristic(s_start[id],s_goal[id], id);
    tmp.cost = C1[id];
    cellHash[s_start[id]] = tmp;
    s_start[id] = calculateKey(s_start[id], id);

    s_last[id] = s_start[id];

    for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
        updateCell(kk->first.x, kk->first.y, kk->second, id);
    }
}
bool Dstar::replan(int id) {

    plans[id].clear();

    int res = computeShortestPath(id);
    
    if (res < 0) {
        fprintf(stderr, "NO PATH TO GOAL\n");
        return false;
    }
    list<state> n;
    list<state>::iterator i;

    state cur = s_start[id];

    if (isinf(getG(s_start[id], id))) {
        fprintf(stderr, "NO PATH TO GOAL\n");
        return false;
    }

    while(cur != s_goal[id]) {
        plans[id].push_back(cur);
        getSucc(cur, n, id);

        if (n.empty()) {
            fprintf(stderr, "NO PATH TO GOAL\n");
            return false;
        }

        double cmin = INFINITY;
        double tmin;
        state smin;

        for (i=n.begin(); i!=n.end(); i++) {
            double val  = cost(cur,*i, id);
            double val2 = trueDist(*i,s_goal[id]) + trueDist(s_start[id],*i); // (Euclidean) cost to goal + cost to pred
            val += getG(*i, id);

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
    plans[id].push_back(s_goal[id]);

    tmpPlans[id] = plans[id];
    
    return true;
}

void Dstar::addEnemyObstacle(const Robot enemy) {
    float cx = enemy.pos.x() / dRatio;
    float cy = enemy.pos.y() / dRatio;
    float outerRadius = 360 / dRatio;
    float innerRadius = 0 / dRatio;

    for (int x = cx - outerRadius; x <= cx + outerRadius; ++x) {
        for (int y = cy - outerRadius; y <= cy + outerRadius; ++y) {
            int dx = x - cx;
            int dy = y - cy;
            int distSq = dx * dx + dy * dy;
            if (distSq <= outerRadius * outerRadius && distSq >= innerRadius * innerRadius) {
                updateCell(x, y, -1, enemy.robotId);
            }
        }
    }
}

void Dstar::addFieldObstacle(int id) {
    for (int i = -(6700 / dRatio); i < (6700 / dRatio); i++) {
      updateCell(i, 5200 / dRatio, -1, id);
      updateCell(i, -5200 / dRatio, -1, id);
    }
    for (int i = -(5200 / dRatio); i < (5200 / dRatio); i++) {
      updateCell(6700 / dRatio, i, -1, id);
      updateCell(-6700 / dRatio, i, -1, id);
    }
}

void Dstar::resetMap()
{
    cellHash.clear();
    for (int i = 0; i < ourIDs.size(); ++i) {
        openHashs[ourIDs[i]].clear();
        while(!openLists[ourIDs[i]].empty()) openLists[ourIDs[i]].pop();
        plans[ourIDs[i]].clear();
        k_ms[ourIDs[i]] = 0;
    }
}

void Dstar::update(Robot* ourRobots, Robot* enemyRobots) {
    std::lock_guard<std::mutex> lock(plansMutex);
    ourIDs.clear();
    enemyIDs.clear();
    for (int i = 0; i < conf["General"]["MaxRobotCount"].as<int>(); ++i) {
        this->ourRobots[i] = ourRobots[i];
        if (ourRobots[i].active) {
            ourIDs.push_back(i);
        }
        this->enemyRobots[i] = enemyRobots[i];
        if (enemyRobots[i].active) {
            enemyIDs.push_back(i);
        }
    }
    
}

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
    for (int i = 0; i < conf["General"]["MaxRobotCount"].as<int>(); i++) {
        for (auto &p : tmpPlans[i]) {
            newPlans[i].push_back(Eigen::Vector2d(p.x*dRatio, p.y*dRatio));
        }
    }
    return newPlans;
}

void Dstar::run() {
    while (running_) {
        // std::cout << "Dstar running" << std::endl;
        resetMap();
        // addFieldObstacle();
        for (int j = 0; j < enemyIDs.size(); j++) {
            if (!enemyRobots[enemyIDs[j]].active) continue;
            addEnemyObstacle(enemyRobots[enemyIDs[j]]);
        }
        for (int i = 0; i < ourIDs.size(); i++) {
            if (!ourRobots[ourIDs[i]].active) continue;
            updateStart(ourRobots[ourIDs[i]]);
            updateGoal(ourRobots[ourIDs[i]]);
            replan(ourIDs[i]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}   
