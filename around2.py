import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 初期位置、ボール位置、ゴール位置
start = np.array([0, 2000])
ball = np.array([4000, 3000])
goal = np.array([3000, 2000])

# ゴール方向の単位ベクトル
goal_direction = (goal - ball)
goal_direction = goal_direction / np.linalg.norm(goal_direction)

# ベジェ曲線の生成関数（4点）
def bezier_curve(p0, p1, p2, p3, t):
    return (1 - t)**3 * p0 + 3 * (1 - t)**2 * t * p1 + 3 * (1 - t) * t**2 * p2 + t**3 * p3

# 2次元ベクトルのクロス積
def cross_2d(v1, v2):
    return v1[0]*v2[1] - v1[1]*v2[0]

# 制御点の計算
def compute_control_points(start, ball, goal_direction):
    v1 = goal - ball
    v2 = start - ball
    angle = np.degrees(np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0)))

    if angle >= 45:
        angle *= 1.3
        rotation_angle = -(90 + angle - 45) if cross_2d(v1, v2) < 0 else (90 + angle - 45)
        radians = np.radians(rotation_angle)
        rotation_matrix = np.array([
            [np.cos(radians), -np.sin(radians)],
            [np.sin(radians),  np.cos(radians)]
        ])
        rotated_direction = rotation_matrix @ goal_direction
        control1 = ball + rotated_direction * 1000
    else:
        rotation = np.array([[0, -1], [1, 0]]) if cross_2d(v1, v2) > 0 else np.array([[0, 1], [-1, 0]])
        perpendicular = rotation @ goal_direction
        control1 = ball + perpendicular * 1000

    distance = 1000.0
    control2 = ball - goal_direction * distance
    if np.linalg.norm(start - ball) < 1000:
        control2 = ball - goal_direction * np.linalg.norm(start - ball)
        control1 = control2

    return control1, control2

# アニメーションの準備
fig, ax = plt.subplots()
robot_dot, = ax.plot([], [], 'go', label='Robot')
curve_line, = ax.plot([], [], 'b-', label='Bezier Curve')
control1_dot, = ax.plot([], [], 'cs', label='Control Point 1')
control2_dot, = ax.plot([], [], 'ms', label='Control Point 2')
path_line, = ax.plot([], [], 'r--', label='Traced Path')  # ロボットがたどった軌跡

ax.plot([start[0]], [start[1]], 'go', label='Start')
ax.plot([ball[0]], [ball[1]], 'ro', label='Ball')
ax.plot([goal[0]], [goal[1]], 'ko', label='Goal')
ax.arrow(ball[0], ball[1], goal_direction[0]*500, goal_direction[1]*500,
         head_width=200, head_length=300, fc='gray', ec='gray')
ax.set_aspect('equal', adjustable='datalim')
ax.grid(True)
ax.set_title('Robot Following Bezier Curve')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.legend()

# ロボットの現在位置を保持
current_pos = start.copy()
path_positions = [current_pos.copy()]  # 軌跡を記録

# アニメーション更新関数
def animate(i):
    global current_pos
    new_control1, new_control2 = compute_control_points(current_pos, ball, goal_direction)
    new_curve = np.array([bezier_curve(current_pos, new_control1, new_control2, ball, t_val) for t_val in np.linspace(0, 1, 100)])

    next_t = 0.01 + (i * 0.01)**100
    current_pos = bezier_curve(current_pos, new_control1, new_control2, ball, next_t)
    path_positions.append(current_pos.copy())

    robot_dot.set_data([current_pos[0]], [current_pos[1]])
    curve_line.set_data(new_curve[:, 0], new_curve[:, 1])
    control1_dot.set_data([new_control1[0]], [new_control1[1]])
    control2_dot.set_data([new_control2[0]], [new_control2[1]])
    path_array = np.array(path_positions)
    path_line.set_data(path_array[:, 0], path_array[:, 1])

    return robot_dot, curve_line, control1_dot, control2_dot, path_line

# アニメーション作成
ani = FuncAnimation(fig, animate, frames=100, interval=100, blit=True)

plt.show()
