import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# 初期位置、ボール位置、ゴール位置
initial_start = np.array([0, 2000])
ball = np.array([4000, 3000])
goal = np.array([3000, 2000])

# ゴール方向の単位ベクトル
goal_direction = (goal - ball)
goal_direction = goal_direction / np.linalg.norm(goal_direction)

# ベジェ曲線の生成関数（4点）
def bezier_curve(p0, p1, p2, p3, t):
    return (1 - t)**3 * p0 + 3 * (1 - t)**2 * t * p1 + 3 * (1 - t) * t**2 * p2 + t**3 * p3

# ベクトル間の角度を計算する関数
def angle_between(v1, v2):
    cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    return np.degrees(np.arccos(cos_theta))

# ゴール方向ベクトルを指定角度だけ回転
def rotate_vector(vec, degrees):
    radians = np.radians(degrees)
    rotation_matrix = np.array([
        [np.cos(radians), -np.sin(radians)],
        [np.sin(radians),  np.cos(radians)]
    ])
    return rotation_matrix @ vec

# 描画関数
def update_plot(start_x, start_y, control1_x, control1_y):
    start = np.array([start_x, start_y])
    control1 = np.array([control1_x, control1_y])

    v1 = goal - ball
    v2 = start - ball
    angle = angle_between(v1, v2)


    if angle >= 45:
        rotation_angle = -(90 + angle - 45) if np.cross(v1, v2) < 0 else (90 + angle - 45)
        rotated_direction = rotate_vector(goal_direction, rotation_angle)
        control1 = ball + rotated_direction * 1000
    else:
        rotation = np.array([[0, -1], [1, 0]]) if np.cross(v1, v2) > 0 else np.array([[0, 1], [-1, 0]])
        perpendicular = rotation @ goal_direction
        control1 = ball + perpendicular * 1000

    distance = 1000.0
    control2 = ball - goal_direction * distance
    if np.linalg.norm(start - ball) < 1000:
        # control2 = ball
        control2 = ball - goal_direction * np.linalg.norm(start - ball)
        control1 = control2
    

    t_values = np.linspace(0, 1, 100)
    curve = np.array([bezier_curve(start, control1, control2, ball, t) for t in t_values])

    ax.clear()
    ax.plot(curve[:, 0], curve[:, 1], 'b-', label='Bezier Curve')
    ax.plot([start[0]], [start[1]], 'go', label='Start Position')
    ax.plot([ball[0]], [ball[1]], 'ro', label='Ball Position')
    ax.plot([goal[0]], [goal[1]], 'ko', label='Goal Position')
    ax.plot([control1[0]], [control1[1]], 'cs', label='Control Point 1')
    ax.plot([control2[0]], [control2[1]], 'ms', label='Control Point 2')
    ax.arrow(ball[0], ball[1], goal_direction[0]*500, goal_direction[1]*500,
             head_width=200, head_length=300, fc='gray', ec='gray')
    ax.legend()
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True)
    ax.set_title('RoboCup SSL: Goal Direction')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

# 初期描画
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.35)
update_plot(initial_start[0], initial_start[1], 0, 0)

# スライダーの設定
ax_start_x = plt.axes([0.25, 0.25, 0.65, 0.03])
ax_start_y = plt.axes([0.25, 0.3, 0.65, 0.03])
slider_start_x = Slider(ax_start_x, 'Start X', -6000.0, 6000.0, valinit=initial_start[0])
slider_start_y = Slider(ax_start_y, 'Start Y', -4500.0, 4500.0, valinit=initial_start[1])

ax_control1_x = plt.axes([0.25, 0.15, 0.65, 0.03])
ax_control1_y = plt.axes([0.25, 0.2, 0.65, 0.03])
slider_control1_x = Slider(ax_control1_x, 'Control1 X', -6000.0, 6000.0, valinit=0)
slider_control1_y = Slider(ax_control1_y, 'Control1 Y', -4500.0, 4500.0, valinit=0)

# スライダーのイベント
def update(val):
    update_plot(slider_start_x.val, slider_start_y.val, slider_control1_x.val, slider_control1_y.val)

slider_start_x.on_changed(update)
slider_start_y.on_changed(update)
slider_control1_x.on_changed(update)
slider_control1_y.on_changed(update)

plt.show()
