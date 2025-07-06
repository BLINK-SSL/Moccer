#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <eigen3/Eigen/Dense>

using Vec2 = Eigen::Vector2d;

struct Obstacle {
    Vec2 center;
    double radius;
};

// 衝突判定：線分p1-p2 と 円 obs
bool check_collision(const Vec2& p1, const Vec2& p2, const Obstacle& obs, double radius_scale=1.0) {
    Vec2 d = p2 - p1;
    Vec2 f = p1 - obs.center;
    double a = d.dot(d);
    double b = 2 * f.dot(d);
    double adjusted_radius = obs.radius * radius_scale;
    double c = f.dot(f) - adjusted_radius * adjusted_radius;

    double discriminant = b*b - 4*a*c;
    if (discriminant < 0) return false;

    double sqrt_disc = std::sqrt(discriminant);
    double t1 = (-b - sqrt_disc) / (2*a);
    double t2 = (-b + sqrt_disc) / (2*a);

    return (0 <= t1 && t1 <= 1) || (0 <= t2 && t2 <= 1);
}

// 中間ターゲット生成
std::vector<Vec2> generate_targets(
    const Vec2& start,
    const Vec2& target,
    int num=60,
    double offset=6000.0,
    int seed = 0)
{
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> angle_dist(-M_PI, M_PI);
    std::uniform_real_distribution<> dist_dist(0.5*offset, 1.0*offset);

    Vec2 vec = target - start;
    Vec2 base = start + 0.5 * vec;
    Vec2 direction = vec.normalized();

    std::vector<Vec2> targets;
    int attempts = 0, max_attempts = 1000;

    while ((int)targets.size() < num && attempts < max_attempts) {
        double angle = angle_dist(gen);
        double distance = dist_dist(gen);

        // 2D回転行列適用
        double c = std::cos(angle);
        double s = std::sin(angle);
        Vec2 offset_vec(c * direction.x() - s * direction.y(),
                        s * direction.x() + c * direction.y());
        offset_vec *= distance;

        Vec2 pt = base + offset_vec;

        Vec2 v1 = pt - start;
        Vec2 v2 = target - pt;
        double cosine_angle = v1.dot(v2) / (v1.norm() * v2.norm());
        cosine_angle = std::clamp(cosine_angle, -1.0, 1.0);
        double angle_between = std::acos(cosine_angle);

        double total_len = v1.norm() + v2.norm();
        double straight_len = (target - start).norm();

        if (angle_between > (160.0 * M_PI / 180.0)) { // 20度未満の曲がりは除外
            attempts++;
            continue;
        }
        if (total_len > straight_len * 1.5) { // 遠回りしすぎ除外
            attempts++;
            continue;
        }

        targets.push_back(pt);
        attempts++;
    }

    return targets;
}

// 経路計画
bool plan_path(
    const Vec2& start,
    const Vec2& target,
    const std::vector<Obstacle>& obstacles,
    Vec2& used_mid)
{
    auto candidates = generate_targets(start, target);

    candidates.push_back(target); // 直進も候補に

    // ゴール方向との角度でソート
    Vec2 v_goal = target - start;
    std::sort(candidates.begin(), candidates.end(), [&](const Vec2& a, const Vec2& b) {
        double cos_a = (v_goal.dot(a - start)) / (v_goal.norm() * (a - start).norm());
        double cos_b = (v_goal.dot(b - start)) / (v_goal.norm() * (b - start).norm());
        cos_a = std::clamp(cos_a, -1.0, 1.0);
        cos_b = std::clamp(cos_b, -1.0, 1.0);
        return std::acos(cos_a) < std::acos(cos_b);
    });

    for (const auto& mid : candidates) {
        bool seg1_collide = false, seg2_collide = false;
        for (const auto& obs : obstacles) {
            if (check_collision(start, mid, obs)) {
                seg1_collide = true;
                break;
            }
        }
        for (const auto& obs : obstacles) {
            if (check_collision(mid, target, obs)) {
                seg2_collide = true;
                break;
            }
        }
        if (!seg1_collide && !seg2_collide) {
            used_mid = mid;
            return true;
        }
    }
    return false;
}

// スムーズ補間（二次スプラインの簡易実装）
std::vector<Vec2> smooth_waypoints(const Vec2& start, const Vec2& mid, const Vec2& target) {
    // ここでは単純に線形補間で代用（C++でmake_interp_spline相当は外部ライブラリ必要）
    // 必要ならEigenのSplineモジュールや他のライブラリを使うと良い

    std::vector<Vec2> traj;
    int points = 200;
    for (int i = 0; i <= points; i++) {
        double t = double(i) / points;
        if (t < 0.5) {
            double local_t = t / 0.5;
            traj.push_back(start * (1 - local_t) + mid * local_t);
        } else {
            double local_t = (t - 0.5) / 0.5;
            traj.push_back(mid * (1 - local_t) + target * local_t);
        }
    }
    return traj;
}

int main() {
    Vec2 start(-6000.0, -4500.0);
    Vec2 target(6000.0, 4500.0);

    std::vector<Obstacle> obstacles = {
        {{3500, 4000}, 180}
    };

    Vec2 current_pos = start;
    Vec2 final_target = target;

    Vec2 used_mid;
    bool has_path = plan_path(current_pos, final_target, obstacles, used_mid);
    if (!has_path) {
        std::cerr << "初期経路が見つかりません\n";
        return -1;
    }

    std::vector<Vec2> smooth_traj = smooth_waypoints(current_pos, used_mid, final_target);
    std::vector<Vec2> path_points = { current_pos };

    int max_steps = 1000;
    double threshold = 200.0;
    int traj_index = 0;
    int step_distance = 10;  // 1ステップで進むインデックス

    for (int step = 0; step < max_steps; ++step) {
        // ゴールに十分近いか？
        if ((current_pos - final_target).norm() < threshold) {
            std::cout << "Step " << step << ": 目標に到達\n";
            break;
        }

        // スムーズ経路の再生成（used_midを使い続ける）
        smooth_traj = smooth_waypoints(current_pos, used_mid, final_target);

        // 次のステップ位置を取得
        int next_index = std::min(step_distance, (int)smooth_traj.size() - 1);
        Vec2 next_pos = smooth_traj[next_index];

        // 衝突チェック
        bool collision = false;
        for (const auto& obs : obstacles) {
            if (check_collision(current_pos, next_pos, obs)) {
                collision = true;
                break;
            }
        }

        if (collision) {
            std::cout << "Step " << step << ": 衝突検出 → 再計画\n";
            bool replan_success = plan_path(current_pos, final_target, obstacles, used_mid);
            if (!replan_success) {
                std::cerr << "再計画失敗。終了\n";
                break;
            }

            // 再計画後、再度スムーズ経路作成
            smooth_traj = smooth_waypoints(current_pos, used_mid, final_target);
            next_index = std::min(step_distance, (int)smooth_traj.size() - 1);
            next_pos = smooth_traj[next_index];
        }

        current_pos = next_pos;
        path_points.push_back(current_pos);
    }

    // 出力
    std::cout << "経路:\n";
    for (const auto& pt : path_points) {
        std::cout << pt.x() << ", " << pt.y() << "\n";
    }

    return 0;
}
