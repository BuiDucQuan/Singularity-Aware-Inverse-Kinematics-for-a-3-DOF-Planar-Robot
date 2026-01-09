import numpy as np

# ===== Robot parameters =====
l1 = 1.0
l2 = 1.0
l3 = 1.0


def fk(q):
    """
    Forward Kinematics for 3-DOF planar robot

    Input:
        q = [q1, q2, q3] (rad)

    Output:
        x, y, phi
    """
    q1, q2, q3 = q

    q12 = q1 + q2
    q123 = q12 + q3

    x = l1 * np.cos(q1) \
      + l2 * np.cos(q12) \
      + l3 * np.cos(q123)

    y = l1 * np.sin(q1) \
      + l2 * np.sin(q12) \
      + l3 * np.sin(q123)

    phi = q123

    return x, y, phi
