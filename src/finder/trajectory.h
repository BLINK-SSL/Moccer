#pragma once

#include "obstacle.h"
#include "../models/robot.h"
#include <vector>
#include <cmath>
#include <limits>

struct Point {
    double x, y;

    Point(double x_, double y_) : x(x_), y(y_) {}
};

class Trajectory {
public:
    std::vector<Robot> points;
    double cost = std::numeric_limits<double>::infinity();

    bool isValid(const std::vector<std::shared_ptr<Obstacle>>& obstacles) const {
        for (const auto& pt : points) {
            for (const auto& obs : obstacles) {
                if (obs->collides(pt.x, pt.y)) return false;
            }
        }
        return true;
    }
};

Trajectory planTrajectory(
    const Robot& start,
    const Point& target,
    const std::vector<std::shared_ptr<CircleObstacle>>& obstacles,
    int num_angles = 20,
    double angle_range = M_PI / 3,    // ±45度範囲
    double step_size = 10.0,          // mm
    int max_steps = 300,
    double goal_tolerance = 20.0     // mm
) {
    Trajectory best_traj;
    double best_cost = std::numeric_limits<double>::infinity();

    double base_angle = std::atan2(target.y - start.y, target.x - start.x);

    for (int i = -num_angles / 2; i <= num_angles / 2; ++i) {
        double offset = i * angle_range / (num_angles / 2);
        double initial_angle = base_angle + offset;

        Robot current = start;
        Trajectory traj;
        double angle = initial_angle;
        const double alpha = 0.01; // ゴール方向への補正率

        for (int step = 0; step < max_steps; ++step) {
            // 現在位置からゴール方向を再計算
            double to_goal_angle = std::atan2(target.y - current.y, target.x - current.x);

            // 元の方向とゴール方向をブレンド
            // angle = (1.0 - alpha) * angle + alpha * to_goal_angle;

            current.x += std::cos(angle) * step_size;
            current.y += std::sin(angle) * step_size;
            current.orientation = angle;

            // 衝突チェック
            bool collision = false;
            for (const auto& obs : obstacles) {
                if (obs->collides(current.x, current.y)) {
                    collision = true;
                    break;
                }
            }
            if (collision) break;

            traj.points.push_back(current);

            // ゴール到達判定
            double dx = target.x - current.x;
            double dy = target.y - current.y;
            double dist = std::hypot(dx, dy);
            if (dist < goal_tolerance) break;
        }

        if (!traj.points.empty()) {
            const auto& last = traj.points.back();
            double cost = std::hypot(target.x - last.x, target.y - last.y);

            if (cost < best_cost) {
                best_cost = cost;
                best_traj = traj;
                best_traj.cost = cost;
            }
        }
    }

    return best_traj;
}
