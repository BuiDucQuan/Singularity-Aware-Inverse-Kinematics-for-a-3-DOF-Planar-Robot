import numpy as np
import matplotlib.pyplot as plt

# =========================
# Jacobian 3-DOF planar (x, y, theta)
# =========================
def jacobian(q, l):
    q1, q2, q3 = q
    l1, l2, l3 = l

    s1 = np.sin(q1)
    c1 = np.cos(q1)
    s12 = np.sin(q1 + q2)
    c12 = np.cos(q1 + q2)
    s123 = np.sin(q1 + q2 + q3)
    c123 = np.cos(q1 + q2 + q3)

    J = np.array([
        [-l1*s1 - l2*s12 - l3*s123,
         -l2*s12 - l3*s123,
         -l3*s123],
        [ l1*c1 + l2*c12 + l3*c123,
          l2*c12 + l3*c123,
          l3*c123],
        [1.0, 1.0, 1.0]
    ])

    return J


# =========================
# IK chuẩn
# =========================
def ik_standard(q, xdot, l):
    J = jacobian(q, l)
    return np.linalg.inv(J) @ xdot


# =========================
# DLS cố định
# =========================
def ik_dls(q, xdot, l, lam=0.05):
    J = jacobian(q, l)
    return J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(3)) @ xdot


# =========================
# Adaptive Damping
# =========================
def adaptive_lambda(J, sigma_th=0.05, lam_max=0.1):
    sigma = np.linalg.svd(J, compute_uv=False)
    sigma_min = np.min(sigma)

    if sigma_min >= sigma_th:
        return 0.0
    else:
        return lam_max * (1 - sigma_min / sigma_th)


def ik_adaptive(q, xdot, l, sigma_th=0.05, lam_max=0.1):
    J = jacobian(q, l)
    lam = adaptive_lambda(J, sigma_th, lam_max)
    return J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(3)) @ xdot


# =========================
# MAIN EXPERIMENT
# =========================
if __name__ == "__main__":

    # Robot parameters
    l = [1.0, 1.0, 1.0]

    # Task velocity (x, y, theta)
    xdot = np.array([0.05, 0.0, 0.0])

    dt = 0.05
    steps = 200

    # Initial configuration (gần singularity)
    q0 = np.array([0.0, 0.6, 0.0])

    # Containers
    qs_std = [q0.copy()]
    qs_dls = [q0.copy()]
    qs_adp = [q0.copy()]

    qdot_std_norm = []
    qdot_dls_norm = []
    qdot_adp_norm = []

    for _ in range(steps):

        # ---- Standard IK ----
        try:
            qdot = ik_standard(qs_std[-1], xdot, l)
            qs_std.append(qs_std[-1] + qdot * dt)
            qdot_std_norm.append(np.linalg.norm(qdot))
        except np.linalg.LinAlgError:
            qs_std.append(qs_std[-1])
            qdot_std_norm.append(np.nan)

        # ---- Fixed DLS ----
        qdot = ik_dls(qs_dls[-1], xdot, l, lam=0.05)
        qs_dls.append(qs_dls[-1] + qdot * dt)
        qdot_dls_norm.append(np.linalg.norm(qdot))

        # ---- Adaptive DLS ----
        qdot = ik_adaptive(qs_adp[-1], xdot, l)
        qs_adp.append(qs_adp[-1] + qdot * dt)
        qdot_adp_norm.append(np.linalg.norm(qdot))

    # =========================
    # Plot velocity norm
    # =========================
    plt.figure(figsize=(8, 5))
    plt.plot(qdot_std_norm, label="Standard IK")
    plt.plot(qdot_dls_norm, label="Fixed DLS")
    plt.plot(qdot_adp_norm, label="Adaptive DLS", linewidth=2)
    plt.xlabel("Time step")
    plt.ylabel("||q̇||")
    plt.title("Joint velocity when passing singularity")
    plt.legend()
    plt.grid(True)
    plt.show()

    # =========================
    # Plot joint trajectory (q2)
    # =========================
    qs_std = np.array(qs_std)
    qs_dls = np.array(qs_dls)
    qs_adp = np.array(qs_adp)

    plt.figure(figsize=(8, 5))
    plt.plot(qs_std[:, 1], label="q2 Standard IK")
    plt.plot(qs_dls[:, 1], label="q2 Fixed DLS")
    plt.plot(qs_adp[:, 1], label="q2 Adaptive DLS")
    plt.xlabel("Time step")
    plt.ylabel("q2")
    plt.title("Joint trajectory through singularity")
    plt.legend()
    plt.grid(True)
    plt.show()
