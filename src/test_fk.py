import numpy as np
import matplotlib.pyplot as plt
from kinematics import fk, l1, l2, l3


def plot_robot(q):
    q1, q2, q3 = q

    # Joint positions
    x0, y0 = 0, 0

    x1 = l1 * np.cos(q1)
    y1 = l1 * np.sin(q1)

    x2 = x1 + l2 * np.cos(q1 + q2)
    y2 = y1 + l2 * np.sin(q1 + q2)

    x3 = x2 + l3 * np.cos(q1 + q2 + q3)
    y3 = y2 + l3 * np.sin(q1 + q2 + q3)

    # Plot robot
    plt.figure()
    plt.plot([x0, x1, x2, x3],
             [y0, y1, y2, y3],
             '-o', linewidth=3)

    plt.axis('equal')
    plt.grid(True)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("3-DOF Planar Robot")

    plt.show()


# ===== TEST =====
q = np.deg2rad([10, 60, -60])

x, y, phi = fk(q)
print("x =", x)
print("y =", y)
print("phi (deg) =", np.rad2deg(phi))

plot_robot(q)
