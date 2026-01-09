import numpy as np
import matplotlib.pyplot as plt

# -------------------------
# Robot parameters
# -------------------------
l1 = l2 = l3 = 1.0
dt = 0.05

# -------------------------
# Kinematics
# -------------------------
def fk(q):
    q1, q2, q3 = q
    x = l1*np.cos(q1) + l2*np.cos(q1+q2) + l3*np.cos(q1+q2+q3)
    y = l1*np.sin(q1) + l2*np.sin(q1+q2) + l3*np.sin(q1+q2+q3)
    return np.array([x, y])

def jacobian(q):
    q1, q2, q3 = q
    s1, c1 = np.sin(q1), np.cos(q1)
    s12, c12 = np.sin(q1+q2), np.cos(q1+q2)
    s123, c123 = np.sin(q1+q2+q3), np.cos(q1+q2+q3)

    J = np.array([
        [-l1*s1 - l2*s12 - l3*s123, -l2*s12 - l3*s123, -l3*s123],
        [ l1*c1 + l2*c12 + l3*c123,  l2*c12 + l3*c123,  l3*c123]
    ])
    return J

# -------------------------
# Desired trajectory
# -------------------------
T = 200
t = np.linspace(0, 2*np.pi, T)
xd = 1.2 + 0.4*np.cos(t)
yd = 0.6 + 0.4*np.sin(t)

# -------------------------
# IK simulation (DLS nhẹ)
# -------------------------
q = np.zeros(3)
traj = []

for k in range(T):
    x = fk(q)
    e = np.array([xd[k], yd[k]]) - x

    J = jacobian(q)
    lam = 0.1
    dq = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(2)) @ e

    q = q + dq * dt
    traj.append(x)

traj = np.array(traj)

# -------------------------
# Plot trajectory
# -------------------------
plt.figure(figsize=(6,6))
plt.plot(traj[:,0], traj[:,1], 'b-', label="End-effector path")
plt.plot(xd, yd, 'k--', label="Desired path")

# -------------------------
# Overlay ellipsoids
# -------------------------
indices = [40, 80, 120, 160]  # vài thời điểm tiêu biểu

for idx in indices:
    q_tmp = np.zeros(3)
    for k in range(idx):
        x = fk(q_tmp)
        e = np.array([xd[k], yd[k]]) - x
        J = jacobian(q_tmp)
        dq = J.T @ np.linalg.inv(J @ J.T + 0.1**2*np.eye(2)) @ e
        q_tmp = q_tmp + dq * dt

    J = jacobian(q_tmp)
    A = J @ J.T
    eigvals, eigvecs = np.linalg.eig(A)

    a, b = np.sqrt(eigvals)
    angle = np.arctan2(eigvecs[1,0], eigvecs[0,0])

    t_ell = np.linspace(0, 2*np.pi, 100)
    ell = np.array([a*np.cos(t_ell), b*np.sin(t_ell)])

    R = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]
    ])

    ell_rot = R @ ell
    center = traj[idx]

    plt.plot(center[0] + 0.2*ell_rot[0],
             center[1] + 0.2*ell_rot[1],
             'r')

# -------------------------
plt.axis("equal")
plt.grid(True)
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()
plt.title("IK Trajectory with Manipulability Ellipsoid Overlay")
plt.show()
