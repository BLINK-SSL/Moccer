import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# 初期位置、ボール位置、ゴール位置
initial_start = np.array([0, 2])
ball = np.array([4, 3])
goal = np.array([3, 2])

# ゴール方向の単位ベクトル
goal_direction = (goal - ball)
goal_direction = goal_direction / np.linalg.norm(goal_direction)

# ベジェ曲線の生成関数（4点）
def bezier_curve(p0, p1, p2, p3, t):
    return (1 - t)**3 * p0 + 3 * (1 - t)**2 * t * p1 + 3 * (1 - t) * t**2 * p2 + t**3 * p3

# 描画関数
def update_plot(start_x, start_y):
    start = np.array([start_x, start_y])

    v1 = goal - ball
    v2 = start - ball
    cross = np.cross(v1, v2)

    if cross > 0:
        rotation = np.array([[0, -1], [1, 0]])  # +90度回転
    else:
        rotation = np.array([[0, 1], [-1, 0]])  # -90度回転

    perpendicular = rotation @ goal_direction
    control1 = ball + perpendicular

    distance = 1.0
    if np.linalg.norm(start - ball) < 1.0:
        distance = np.linalg.norm(start - ball)
    control2 = ball - goal_direction * distance

    t_values = np.linspace(0, 1, 100)
    curve = np.array([bezier_curve(start, control1, control2, ball, t) for t in t_values])

    ax.clear()
    ax.plot(curve[:, 0], curve[:, 1], 'b-', label='Bezier Curve')
    ax.plot([start[0]], [start[1]], 'go', label='Start Position')
    ax.plot([ball[0]], [ball[1]], 'ro', label='Ball Position')
    ax.plot([goal[0]], [goal[1]], 'ko', label='Goal Position')
    ax.plot([control1[0]], [control1[1]], 'cs', label='Control Point 1')
    ax.plot([control2[0]], [control2[1]], 'ms', label='Control Point 2')
    ax.arrow(ball[0], ball[1], goal_direction[0], goal_direction[1],
             head_width=0.2, head_length=0.3, fc='gray', ec='gray')
    ax.legend()
    ax.axis('equal')
    ax.grid(True)
    ax.set_title('RoboCup SSL: Goal Direction')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    fig.canvas.draw_idle()

# 初期描画
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.25)
update_plot(initial_start[0], initial_start[1])

# スライダーの設定
ax_start_x = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_start_y = plt.axes([0.25, 0.15, 0.65, 0.03])
slider_start_x = Slider(ax_start_x, 'Start X', -5.0, 5.0, valinit=initial_start[0])
slider_start_y = Slider(ax_start_y, 'Start Y', -5.0, 5.0, valinit=initial_start[1])

# スライダーのイベント
def update(val):
    update_plot(slider_start_x.val, slider_start_y.val)

slider_start_x.on_changed(update)
slider_start_y.on_changed(update)

plt.show()
