# import math
# import random
# import matplotlib.pyplot as plt

# # フィールドの範囲
# FIELD_X_MIN, FIELD_Y_MIN = -6000, -4500
# FIELD_X_MAX, FIELD_Y_MAX = 6000, 4500

# # 始点と終点
# current_position = (FIELD_X_MIN, FIELD_Y_MIN)
# target_position = (FIELD_X_MAX, FIELD_Y_MAX)

# # 障害物の位置（フィールド内に配置）
# obstacles = [
#     (random.uniform(5000, FIELD_X_MAX-200), random.uniform(3500, FIELD_Y_MAX-200))
#     for _ in range(8)
# ]

# # 軌道を生成する関数（直線的な軌道）
# def generate_trajectory(start, end, steps=20):
#     path = []
#     for i in range(steps + 1):
#         x = start[0] + (end[0] - start[0]) * i / steps
#         y = start[1] + (end[1] - start[1]) * i / steps
#         path.append((x, y))
#     return path

# # 軌道が障害物を避けているか判定
# def is_trajectory_acceptable(path):
#     for point in path:
#         for obs in obstacles:
#             if math.dist(point, obs) < 300:
#                 return False
#     return True

# # 中間目標をランダムに生成（フィールド内）
# def generate_intermediate_targets(n=10):
#     targets = []
#     for _ in range(n):
#         x = random.uniform(FIELD_X_MIN, FIELD_X_MAX)
#         y = random.uniform(FIELD_Y_MIN, FIELD_Y_MAX)
#         targets.append((x, y))
#     return targets

# # 角度を計算する関数（現在→中間と現在→目標のベクトルのなす角）
# def angle_to_target(current, mid, target):
#     def vector(a, b):
#         return (b[0] - a[0], b[1] - a[1])
#     v1 = vector(current, mid)
#     v2 = vector(current, target)
#     dot = v1[0]*v2[0] + v1[1]*v2[1]
#     norm1 = math.hypot(*v1)
#     norm2 = math.hypot(*v2)
#     return math.acos(dot / (norm1 * norm2 + 1e-6))

# # メイン処理（2段階中間目標）
# def plan_path_with_two_midpoints():
#     for try_num in range(100):
#         direct_path = generate_trajectory(current_position, target_position)
#         if is_trajectory_acceptable(direct_path):
#             return direct_path, "Direct", [], []

#         intermediate_targets = generate_intermediate_targets()
#         intermediate_targets.sort(key=lambda mid: angle_to_target(current_position, mid, target_position))

#         for i in range(len(intermediate_targets)):
#             for j in range(len(intermediate_targets)):
#                 if i == j:
#                     continue
#                 mid1 = intermediate_targets[i]
#                 mid2 = intermediate_targets[j]

#                 leg1 = generate_trajectory(current_position, mid1)
#                 if not is_trajectory_acceptable(leg1):
#                     continue
#                 leg2 = generate_trajectory(mid1, mid2)
#                 if not is_trajectory_acceptable(leg2):
#                     continue
#                 leg3 = generate_trajectory(mid2, target_position)
#                 full_path = leg1 + leg2 + leg3
#                 if is_trajectory_acceptable(full_path):
#                     print(f"Path found after {try_num + 1} attempts")
#                     return full_path, "Two Intermediate", intermediate_targets, [mid1, mid2]
#     return None, "None", intermediate_targets, []

# # 軌道の計画と可視化
# path, path_type, intermediate_targets, selected_midpoints = plan_path_with_two_midpoints()

# plt.figure(figsize=(10, 8))
# plt.plot(current_position[0], current_position[1], 'go', label='Current Position')
# plt.plot(target_position[0], target_position[1], 'ro', label='Target Position')

# # 障害物の描画
# for obs in obstacles:
#     plt.plot(obs[0], obs[1], 'ks', label='Obstacle' if obs == obstacles[0] else "")

# # 中間目標の描画
# for idx, mid in enumerate(intermediate_targets):
#     plt.plot(mid[0], mid[1], 'c^', label='Intermediate Target' if idx == 0 else "")
#     plt.text(mid[0]+100, mid[1]+100, f'M{idx+1}', fontsize=9)

# # 採用された中間目標の強調表示
# for idx, mid in enumerate(selected_midpoints):
#     plt.plot(mid[0], mid[1], 'm*', markersize=12, label=f'Selected Midpoint {idx+1}')

# # 軌道の描画
# if path:
#     x_vals, y_vals = zip(*path)
#     plt.plot(x_vals, y_vals, 'b-', label=f'{path_type} Path')
# else:
#     plt.title("No acceptable path found")

# plt.legend()
# plt.grid(True)
# plt.xlabel("X")
# plt.ylabel("Y")
# plt.title("Trajectory Planning with Two Intermediate Waypoints")
# plt.axis("equal")
# plt.xlim(FIELD_X_MIN - 500, FIELD_X_MAX + 500)
# plt.ylim(FIELD_Y_MIN - 500, FIELD_Y_MAX + 500)
# plt.show()

def ex9_4(data1, data2):
    result = []
    i1 = 0
    i2 = 0
    while i1 < len(data1) and i2 < len(data2):
        if data1[i1] == data2[i2]:
            result.append(data1[i1])
            i1 = i1 + 1
            i2 = i2 + 1
        elif data1[i1] < data2[i2]:
            i1 = i1 + 1
        else:
            i2 = i2 + 1
    return result

print(ex9_4([1, 2, 3, 4, 5], [3, 4, 5, 6, 7]))  # Output: [3, 4, 5]