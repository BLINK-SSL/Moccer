#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <tuple>
#include <chrono>
#include <thread>
#include "src/observer.h"

using namespace std::chrono;

struct Vec2 {
    double x, y;
    Vec2(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
    Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
    Vec2 operator*(double scalar) const { return Vec2(x * scalar, y * scalar); }
    double norm() const { return std::sqrt(x * x + y * y); }
    double distance(const Vec2& other) const { return (*this - other).norm(); }
};

struct Obstacle {
    Vec2 center;
    double radius;
};

bool check_collision(const Vec2& p1, const Vec2& p2, const Obstacle& obs, double radius_scale = 1) {
    Vec2 d = p2 - p1;
    Vec2 f = p1 - obs.center;

    double a = d.x * d.x + d.y * d.y;
    double b = 2 * (f.x * d.x + f.y * d.y);
    double r = obs.radius * radius_scale;
    double c = f.x * f.x + f.y * f.y - r * r;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false;

    discriminant = std::sqrt(discriminant);
    double t1 = (-b - discriminant) / (2 * a);
    double t2 = (-b + discriminant) / (2 * a);
    return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
}

std::vector<Vec2> generate_targets(const Vec2& start, const Vec2& target, int num = 1000, double offset = 6000.0, int max_attempts = 1000) {
    std::vector<Vec2> targets;
    Vec2 vec = target - start;
    Vec2 base = start + vec * 0.5;
    Vec2 direction = vec * (1.0 / vec.norm());
    int attempts = 0;

    while (targets.size() < num && attempts < max_attempts) {
        double angle = ((double)rand() / RAND_MAX) * 2 * M_PI - M_PI;
        double dist = ((double)rand() / RAND_MAX) * 0.5 + 0.5;
        dist *= offset;

        double cs = std::cos(angle);
        double sn = std::sin(angle);
        Vec2 offset_vec(direction.x * cs - direction.y * sn, direction.x * sn + direction.y * cs);
        Vec2 pt = base + offset_vec * dist;

        Vec2 v1 = pt - start;
        Vec2 v2 = target - pt;
        double dot = (v1.x * v2.x + v1.y * v2.y) / (v1.norm() * v2.norm());
        dot = std::max(-1.0, std::min(1.0, dot));
        double angle_between = std::acos(dot);

        double total_len = v1.norm() + v2.norm();
        double straight_len = (target - start).norm();

        if (angle_between > M_PI * 160 / 180.0 || total_len > straight_len * 1.5) {
            ++attempts;
            continue;
        }

        targets.push_back(pt);
        ++attempts;
    }
    return targets;
}

std::pair<Vec2, std::vector<Vec2>> plan_path(const Vec2& start, const Vec2& target, const std::vector<Obstacle>& obstacles) {
    std::vector<Vec2> candidates = generate_targets(start, target);
    candidates.push_back(target);

    std::sort(candidates.begin(), candidates.end(), [&](const Vec2& a, const Vec2& b) {
        Vec2 va = a - start;
        Vec2 vb = b - start;
        Vec2 vg = target - start;
        double da = std::acos(std::max(-1.0, std::min(1.0, (va.x * vg.x + va.y * vg.y) / (va.norm() * vg.norm()))));
        double db = std::acos(std::max(-1.0, std::min(1.0, (vb.x * vg.x + vb.y * vg.y) / (vb.norm() * vg.norm()))));
        return da < db;
    });

    for (const auto& mid : candidates) {
        bool seg1 = false, seg2 = false;
        for (const auto& obs : obstacles) {
            if (check_collision(start, mid, obs)) seg1 = true;
            if (check_collision(mid, target, obs)) seg2 = true;
            if (seg1 || seg2) break;
        }
        if (!seg1 && !seg2) return {mid, candidates};
    }
    return {Vec2(), candidates};
}

std::vector<Vec2> quadratic_spline(const Vec2& p0, const Vec2& p1, const Vec2& p2, int num_points = 200) {
    std::vector<Vec2> result;
    result.reserve(num_points);
    for (int i = 0; i < num_points; ++i) {
        double t = (double)i / (num_points - 1);
        double b0 = (1 - t) * (1 - t);
        double b1 = 2 * t * (1 - t);
        double b2 = t * t;
        double x = b0 * p0.x + b1 * p1.x + b2 * p2.x;
        double y = b0 * p0.y + b1 * p1.y + b2 * p2.y;
        result.emplace_back(x, y);
    }
    return result;
}

int main() {
    Observer observer;
    Vec2 target(6000.0, 4500.0);
    auto last_send_time = steady_clock::now();
    int step_size = 10;
    int threshold = 200;

    Vec2 previous_mid(0, 0);  // 直前のmidを保持
    bool mid_valid = false;
    while (true) {
        observer.update();

        // 現在位置取得
        Robot startRobot = observer.receiver.getBlueRobots()[0];
        Vec2 current_pos(startRobot.x, startRobot.y);

        // 障害物定義
        Robot yellowRobot = observer.receiver.getYellowRobots()[0];
        Robot yellowRobot2 = observer.receiver.getYellowRobots()[1];
        Robot yellowRobot3 = observer.receiver.getYellowRobots()[2];
        std::vector<Obstacle> obstacles = {
            {{yellowRobot.x, yellowRobot.y}, 180},
            {{yellowRobot2.x, yellowRobot2.y}, 180},
            {{yellowRobot3.x, yellowRobot3.y}, 180}
        };

        // 前回のmidが有効か確認
        // if (previous_mid.x != 0 || previous_mid.y != 0) {
        //     mid_valid = true;
        // }
        
        for (const auto& obs : obstacles) {
            if (check_collision(current_pos, previous_mid, obs) ||
                check_collision(previous_mid, target, obs)) {
                mid_valid = false;
                std::cout << "前回のmidが無効 → 再計画\n";
                break;
            }
        }

        Vec2 mid;
        if (mid_valid) {
            mid = previous_mid;
        } else {
            auto [new_mid, _] = plan_path(current_pos, target, obstacles);
            if (new_mid.x == 0 && new_mid.y == 0) {
                std::cerr << "再計画失敗\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            mid = new_mid;
            previous_mid = mid;
        }

        std::vector<Vec2> traj = quadratic_spline(current_pos, mid, target);
        int next_index = std::min(step_size, (int)traj.size() - 1);
        Vec2 next_pos = traj[next_index];

        // 衝突チェック
        bool collision = false;
        for (const auto& obs : obstacles) {
            if (check_collision(current_pos, next_pos, obs)) {
                collision = true;
                break;
            }
        }

        if (!collision) {
            double theta = std::atan2(next_pos.y - current_pos.y, next_pos.x - current_pos.x);
            if (duration_cast<milliseconds>(steady_clock::now() - last_send_time).count() >= 16) {
                observer.sender.send(false, theta);
                last_send_time = steady_clock::now();
            }
        } else {
            std::cout << "衝突検出 → 再計画\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    return 0;
}
