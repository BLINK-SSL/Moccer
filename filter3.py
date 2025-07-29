import math
import random
import matplotlib.pyplot as plt

random.seed(49)  # 固定シードで再現性を確保

# フィールドの範囲
FIELD_X_MIN, FIELD_Y_MIN = -6000, -4500
FIELD_X_MAX, FIELD_Y_MAX = 6000, 4500

# 始点と終点
current_position = (FIELD_X_MIN, FIELD_Y_MIN)
target_position = (FIELD_X_MAX, FIELD_Y_MAX)

# 障害物の半径
obstacle_radius = 90

# 障害物の位置（フィールド内に配置）
obstacles = [
    (random.uniform(5000, FIELD_X_MAX - 200), random.uniform(3500, FIELD_Y_MAX - 200))
    for _ in range(16)
]

# キャッシュされた成功中間地点（例として1つ追加）
cached_successful_midpoints = [(-2000, -1500)]

# 許容できる軌道かどうかを判定する関数
def is_trajectory_acceptable(path):
    for point in path:
        for obs in obstacles:
            if math.dist(point, obs) < obstacle_radius:  # 障害物との距離が近すぎる場合はNG
                return False
    return True

# 軌道を生成する関数（直線的な軌道）
def generate_trajectory(start, end, steps=60):
    path = []
    for i in range(steps + 1):
        x = start[0] + (end[0] - start[0]) * i / steps
        y = start[1] + (end[1] - start[1]) * i / steps
        path.append((x, y))
    return path

# 直線軌道の式を生成する関数
def line_equation(start, end):
    a = (end[1] - start[1]) / (end[0] - start[0]) if end[0] != start[0] else float('inf')
    b = start[1] - a * start[0]
    return a, b

# 中間目標をランダムに生成（フィールド内）
def generate_intermediate_targets(n=5):
    targets = []
    line = line_equation(current_position, target_position)
    vertical_line = (-1 / line[0], line[1])
    # 垂直な直線の方程式の均等に分布した点を生成
    for _ in range(n):
        x = random.uniform(FIELD_X_MIN, FIELD_X_MAX)
        y = vertical_line[0] * x + vertical_line[1]
        targets.append((x, y))
    # for _ in range(n):
    #     x = random.uniform(FIELD_X_MIN, FIELD_X_MAX)
    #     y = random.uniform(FIELD_Y_MIN, FIELD_Y_MAX)
    #     targets.append((x, y))
    return targets

# 角度を計算する関数（現在→中間と現在→目標のベクトルのなす角）
def angle_to_target(current, mid, target):
    def vector(a, b):
        return (b[0] - a[0], b[1] - a[1])
    v1 = vector(current, mid)
    v2 = vector(current, target)
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    norm1 = math.hypot(*v1)
    norm2 = math.hypot(*v2)
    return math.acos(dot / (norm1 * norm2 + 1e-6))  # 小さな値でゼロ除算防止

# メイン処理
def plan_path():
    direct_path = generate_trajectory(current_position, target_position)

    if is_trajectory_acceptable(direct_path):
        return direct_path, "Direct", [], None

    # ランダム中間目標生成 + キャッシュ追加
    random_targets = generate_intermediate_targets()
    all_targets = cached_successful_midpoints + random_targets

    # 角度でソート（目標方向に近い順）
    all_targets.sort(key=lambda mid: angle_to_target(current_position, mid, target_position))

    for mid in all_targets:
        first_leg = generate_trajectory(current_position, mid)
        if not is_trajectory_acceptable(first_leg):
            continue
        second_leg = generate_trajectory(mid, target_position)
        full_path = first_leg + second_leg
        if is_trajectory_acceptable(full_path):
            return full_path, "Intermediate", all_targets, mid
    return None, "None", all_targets, None

# 軌道の計画と可視化
path, path_type, intermediate_targets, selected_midpoint = plan_path()

plt.figure(figsize=(10, 8))
plt.plot(current_position[0], current_position[1], 'go', label='Current Position')
plt.plot(target_position[0], target_position[1], 'ro', label='Target Position')

# 障害物の描画（影響範囲を円で表示）
for obs in obstacles:
    circle = plt.Circle(obs, obstacle_radius, color='gray', alpha=0.3)
    plt.gca().add_patch(circle)
    plt.plot(obs[0], obs[1], 'ks', label='Obstacle' if obs == obstacles[0] else "")

# 中間目標の描画
for idx, mid in enumerate(intermediate_targets):
    plt.plot(mid[0], mid[1], 'c^', label='Intermediate Target' if idx == 0 else "")
    plt.text(mid[0]+100, mid[1]+100, f'M{idx+1}', fontsize=9)

# 採用された中間目標の強調表示
if selected_midpoint:
    plt.plot(selected_midpoint[0], selected_midpoint[1], 'm*', markersize=12, label='Selected Midpoint')

# 軌道の描画
if path:
    x_vals, y_vals = zip(*path)
    plt.plot(x_vals, y_vals, 'b-', label=f'{path_type} Path')
else:
    plt.title("No acceptable path found")

plt.legend()
plt.grid(True)
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Trajectory Planning with Obstacle Influence Range")
plt.axis("equal")
plt.xlim(FIELD_X_MIN - 500, FIELD_X_MAX + 500)
plt.ylim(FIELD_Y_MIN - 500, FIELD_Y_MAX + 500)
plt.show()
