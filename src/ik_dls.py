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

# =========================
# Baseline IK (inv)
# =========================
def ik_standard(q, xdot, l):
    J = jacobian(q, l)
    return np.linalg.inv(J) @ xdot


# =========================
# Damped Least Squares IK
# qdot = J^T (J J^T + λ^2 I)^(-1) xdot
# =========================
def ik_dls(q, xdot, l, lam):
    J = jacobian(q, l)
    m = J.shape[0]
    return J.T @ np.linalg.inv(J @ J.T + (lam**2) * np.eye(m)) @ xdot


# =========================
# MAIN EXPERIMENT
# =========================
if __name__ == "__main__":

    l = [1.0, 1.0, 1.0]
    xdot = np.array([0.05, 0.0,0.0])

    q1 = 0.0
    q3 = 0.0
    q2_vals = np.linspace(0.6, 0.01, 150)

    qdot_std = []
    qdot_dls_1 = []
    qdot_dls_2 = []
    qdot_dls_3 = []

    for q2 in q2_vals:
        q = np.array([q1, q2, q3])

        # Standard IK
        try:
            qd = ik_standard(q, xdot, l)
            qdot_std.append(np.linalg.norm(qd))
        except np.linalg.LinAlgError:
            qdot_std.append(np.nan)

        # DLS with different lambdas
        qdot_dls_1.append(np.linalg.norm(ik_dls(q, xdot, l, lam=0.01)))
        qdot_dls_2.append(np.linalg.norm(ik_dls(q, xdot, l, lam=0.05)))
        qdot_dls_3.append(np.linalg.norm(ik_dls(q, xdot, l, lam=0.1)))

    # =========================
    # PLOT COMPARISON
    # =========================
    plt.figure(figsize=(8, 5))
    plt.plot(q2_vals, qdot_std, label="Standard IK (inv)", linewidth=2)
    plt.plot(q2_vals, qdot_dls_1, label="DLS λ=0.01")
    plt.plot(q2_vals, qdot_dls_2, label="DLS λ=0.05")
    plt.plot(q2_vals, qdot_dls_3, label="DLS λ=0.1")

    plt.xlabel("q2 → 0 (approaching singularity)")
    plt.ylabel("||q̇||")
    plt.title("Standard IK vs DLS near singularity")
    plt.legend()
    plt.grid(True)
    plt.show()
