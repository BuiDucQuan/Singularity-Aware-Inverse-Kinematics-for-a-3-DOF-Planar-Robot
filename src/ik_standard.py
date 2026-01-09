import numpy as np
import matplotlib.pyplot as plt

def jacobian(q, l=[1.0, 1.0, 1.0]):
    q1, q2, q3 = q
    l1, l2, l3 = l

    s1 = np.sin(q1)
    s12 = np.sin(q1 + q2)
    s123 = np.sin(q1 + q2 + q3)

    c1 = np.cos(q1)
    c12 = np.cos(q1 + q2)
    c123 = np.cos(q1 + q2 + q3)

    J = np.array([
        [-l1*s1 - l2*s12 - l3*s123,
         -l2*s12 - l3*s123,
         -l3*s123],
        [ l1*c1 + l2*c12 + l3*c123,
          l2*c12 + l3*c123,
          l3*c123],
        [1,1,1]
    ])
    return J

def ik_differential(q, xdot, l):
    J = jacobian(q, l)
    qdot = np.linalg.inv(J) @ xdot
    return qdot


if __name__ == "__main__":

    l = [1.0, 1.0, 1.0]
    xdot = np.array([0.05, 0.0,0.0])

    q1 = 0.0
    q2_vals = np.linspace(0.6, 0.01, 150)
    q3 = 0.0

    qdot_norms = []

    for q2 in q2_vals:         
        q = np.array([q1, q2, q3])

        try:
            qdot = ik_differential(q, xdot, l)
            qdot_norms.append(np.linalg.norm(qdot))
        except np.linalg.LinAlgError:

            qdot_norms.append(np.nan)

    plt.figure()
    plt.plot(q2_vals, qdot_norms)
    plt.xlabel("q2 → 0 (approaching singularity)")
    plt.ylabel("||q̇||")
    plt.title("Baseline IK: Joint velocity explosion near singularity")
    plt.grid()
    plt.show()
