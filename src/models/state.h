#pragma once

#include <list>

using namespace std;

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
