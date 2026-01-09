import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ======================
# Robot parameters
# ======================
l = np.array([1.0, 1.0, 0.7])
dt = 0.05
lam = 0.4
Kp = 2.0

# ======================
# Forward Kinematics
# ======================
def fk(q):
    q1, q2, q3 = q
    x = l[0]*np.cos(q1) + l[1]*np.cos(q1+q2) + l[2]*np.cos(q1+q2+q3)
    y = l[0]*np.sin(q1) + l[1]*np.sin(q1+q2) + l[2]*np.sin(q1+q2+q3)
    return np.array([x, y])

# ======================
# Jacobian 2x3
# ======================
def jacobian_xy(q):
    q1, q2, q3 = q
    s1, s12, s123 = np.sin([q1, q1+q2, q1+q2+q3])
    c1, c12, c123 = np.cos([q1, q1+q2, q1+q2+q3])

    J = np.array([
        [-l[0]*s1 - l[1]*s12 - l[2]*s123,
         -l[1]*s12 - l[2]*s123,
         -l[2]*s123],
        [ l[0]*c1 + l[1]*c12 + l[2]*c123,
          l[1]*c12 + l[2]*c123,
          l[2]*c123]
    ])
    return J

# ======================
# DLS IK (fast)
# ======================
def ik_dls_xy(q, xdot):
    J = jacobian_xy(q)
    A = J @ J.T + (lam**2) * np.eye(2)
    return J.T @ np.linalg.solve(A, xdot)

# ======================
# Trajectory
# ======================
T = 200
t = np.linspace(0, 2*np.pi, T)

xd = 1.2 + 0.4*np.cos(t)
yd = 0.6 + 0.4*np.sin(t)
xd_dot = -0.4*np.sin(t)
yd_dot =  0.4*np.cos(t)

# ======================
# State storage
# ======================
q = np.array([0.1, 0.1, -0.1])
ee_traj = np.zeros((T, 2))

# ======================
# Plot setup
# ======================
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.grid()

robot_line, = ax.plot([], [], "o-", lw=3, color="tab:blue", label="Robot configuration")
traj_line, = ax.plot([], [], "r--", lw=1, label="Actual end-effector path")
target_line, = ax.plot(xd, yd, "k:", lw=1, label="Desired trajectory")

# ======================
# Update
# ======================
def update(frame):
    global q

    x = fk(q)
    e = np.array([xd[frame], yd[frame]]) - x
    xdot = np.array([xd_dot[frame], yd_dot[frame]]) + Kp * e
    qdot = ik_dls_xy(q, xdot)

    q += qdot * dt

    q1, q2, q3 = q
    p0 = np.array([0, 0])
    p1 = np.array([l[0]*np.cos(q1), l[0]*np.sin(q1)])
    p2 = p1 + np.array([l[1]*np.cos(q1+q2), l[1]*np.sin(q1+q2)])
    p3 = p2 + np.array([l[2]*np.cos(q1+q2+q3), l[2]*np.sin(q1+q2+q3)])

    ee_traj[frame] = p3

    robot_line.set_data(
        [p0[0], p1[0], p2[0], p3[0]],
        [p0[1], p1[1], p2[1], p3[1]]
    )
    traj_line.set_data(ee_traj[:frame+1,0], ee_traj[:frame+1,1])

    return robot_line, traj_line




# ======================
# Animation
# ======================
ani = FuncAnimation(
    fig, update,
    frames=T,
    interval=30,
    blit=True
)
ani.save("demo_2.gif", writer="pillow", fps=20)
plt.show()
