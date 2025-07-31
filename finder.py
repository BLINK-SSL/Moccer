import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

# --- パラメータ設定 ---
start = np.array([-6000.0, -4500.0])
target = np.array([6000.0, 4500.0])
radius = 180
seed = 44
obstacles = [
    {"center": np.array([5500, 4000]), "radius": radius},
    {"center": np.array([3500, 1000]), "radius": radius},
    {"center": np.array([0, 400]), "radius": radius},
    {"center": np.array([4000, 2000]), "radius": radius},
    {"center": np.array([3000, 4500]), "radius": radius},
    {"center": np.array([4500, 2500]), "radius": radius},
    {"center": np.array([-2000, 3000]), "radius": radius},
    {"center": np.array([-4000, 1000]), "radius": radius},
    {"center": np.array([-5000, -2000]), "radius": radius},
    {"center": np.array([-3000, -4000]), "radius": radius},
    {"center": np.array([1000, -3000]), "radius": radius},
    {"center": np.array([2000, -1000]), "radius": radius},
    {"center": np.array([-1000, 2000]), "radius": radius},
    {"center": np.array([-1500, -1500]), "radius": radius},
    {"center": np.array([2500, 3500]), "radius": radius},
    {"center": np.array([4500, -3000]), "radius": radius},
    {"center": np.array([-3500, -2500]), "radius": radius},
    {"center": np.array([3000, 1000]), "radius": radius},
    {"center": np.array([-500, -500]), "radius": radius}
]

def check_collision(p1, p2, obs, radius_scale=1.0):
    d = p2 - p1
    f = p1 - obs["center"]
    a = np.dot(d, d)
    b = 2 * np.dot(f, d)
    adjusted_radius = obs["radius"] * radius_scale
    c = np.dot(f, f) - adjusted_radius ** 2
    discriminant = b ** 2 - 4 * a * c
    if discriminant < 0:
        return False
    sqrt_disc = np.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2 * a)
    t2 = (-b + sqrt_disc) / (2 * a)
    return (0 <= t1 <= 1) or (0 <= t2 <= 1)

def generate_targets(start, target, num=10, offset=6000.0, seed=None, max_attempts=1000):
    if seed is not None:
        np.random.seed(seed)

    vec = target - start
    base = start + vec * 0.5
    direction = vec / np.linalg.norm(vec)

    targets = []
    attempts = 0

    while len(targets) < num and attempts < max_attempts:
        angle = np.random.uniform(-np.pi, np.pi)
        distance = np.random.uniform(0.5, 1.0) * offset

        rot_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle),  np.cos(angle)]
        ])
        offset_vec = rot_matrix @ direction * distance
        pt = base + offset_vec

        v1 = pt - start
        v2 = target - pt
        angle_between = np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0))

        total_len = np.linalg.norm(v1) + np.linalg.norm(v2)
        straight_len = np.linalg.norm(target - start)

        if angle_between > np.radians(160):
            attempts += 1
            continue
        if total_len > straight_len * 1.5:
            attempts += 1
            continue

        targets.append(pt)
        attempts += 1

    return targets

def plan_path(start, target, obstacles):
    candidates = generate_targets(start, target, num=100, seed=seed)
    candidates.append(target)

    def angle_from_goal(mid):
        v_goal = target - start
        v_mid = mid - start
        cos_angle = np.dot(v_goal, v_mid) / (np.linalg.norm(v_goal) * np.linalg.norm(v_mid))
        return np.arccos(np.clip(cos_angle, -1.0, 1.0))

    candidates.sort(key=angle_from_goal)

    for mid in candidates:
        seg1_collide = any(check_collision(start, mid, o) for o in obstacles)
        seg2_collide = any(check_collision(mid, target, o) for o in obstacles)
        if not seg1_collide and not seg2_collide:
            return mid, candidates
    return None, candidates

def smooth_waypoints(start, mid, target):
    waypoints = np.array([start, mid, target])
    t = [0, 0.5, 1.0]
    x_spline = make_interp_spline(t, waypoints[:, 0], k=2)
    y_spline = make_interp_spline(t, waypoints[:, 1], k=2)

    t_dense = np.linspace(0, 1, 200)
    x_dense = x_spline(t_dense)
    y_dense = y_spline(t_dense)
    return np.column_stack([x_dense, y_dense])

current_pos = start.copy()
final_target = target.copy()

path_points = [current_pos]
all_candidates = []  # ★ 中間候補記録用

# 最初の計画
used_mid, candidates = plan_path(current_pos, final_target, obstacles)
if used_mid is None:
    raise RuntimeError("初期経路が見つかりません")
all_candidates.extend(candidates)

smooth_traj = smooth_waypoints(current_pos, used_mid, final_target)
step_size = 10
traj_index = 0

max_steps = 100
threshold = 200

for step in range(max_steps):
    next_index = min(traj_index + step_size, len(smooth_traj) - 1)
    next_pos = smooth_traj[next_index]

    collision = any(check_collision(current_pos, next_pos, obs) for obs in obstacles)

    if collision:
        print(f"Step {step}: 衝突検出。再計画実施")
        used_mid, candidates = plan_path(current_pos, final_target, obstacles)
        if used_mid is None:
            print("再計画失敗。終了")
            break
        all_candidates.extend(candidates)  # ★ 新しい候補を追加
        smooth_traj = smooth_waypoints(current_pos, used_mid, final_target)
        traj_index = 0
        next_index = min(step_size, len(smooth_traj) - 1)
        next_pos = smooth_traj[next_index]
    else:
        traj_index = next_index

    current_pos = next_pos
    path_points.append(current_pos)

    if np.linalg.norm(current_pos - final_target) < threshold:
        print(f"Step {step}: 目標に到達")
        break

# --- 描画 ---
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(-8000, 8000)
ax.set_ylim(-7000, 7000)
ax.set_aspect('equal')
ax.set_title('Incremental Path Following with Replanning')

for obs in obstacles:
    circle = plt.Circle(obs["center"], obs["radius"], color='gray', alpha=0.5)
    ax.add_patch(circle)

# Path & Points
path_points = np.array(path_points)
ax.plot(path_points[:, 0], path_points[:, 1], 'b.-', label='Path')
ax.plot(path_points[0, 0], path_points[0, 1], 'go', label='Start')
ax.plot(*final_target, 'ro', label='Target')

# ★ 全中間ターゲット候補
if all_candidates:
    all_candidates_np = np.array(all_candidates)
    ax.plot(all_candidates_np[:, 0], all_candidates_np[:, 1], 'kx', alpha=0.3, label='Candidates')

ax.legend()
ax.grid(True)
plt.show()
