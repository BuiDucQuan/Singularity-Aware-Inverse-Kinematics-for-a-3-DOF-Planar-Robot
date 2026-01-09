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
# Compute adaptive lambda
# =========================
def adaptive_lambda(J, sigma_th=0.05, lam_max=0.1):
    # Singular values of J
    sigma = np.linalg.svd(J, compute_uv=False)
    sigma_min = np.min(sigma)

    if sigma_min >= sigma_th:
        return 0.0, sigma_min
    else:
        lam = lam_max * (1.0 - sigma_min / sigma_th)
        return lam, sigma_min


# =========================
# Adaptive DLS IK
# qdot = J^T (J J^T + λ^2 I)^(-1) xdot
# =========================
def ik_adaptive_dls(q, xdot, l, sigma_th=0.05, lam_max=0.1):
    J = jacobian(q, l)
    lam, sigma_min = adaptive_lambda(J, sigma_th, lam_max)

    m = J.shape[0]   # task dimension (3 if full Jacobian)
    qdot = J.T @ np.linalg.inv(J @ J.T + (lam**2) * np.eye(m)) @ xdot

    return qdot, lam, sigma_min


# =========================
# MAIN EXPERIMENT
# =========================
if __name__ == "__main__":

    l = [1.0, 1.0, 1.0]

    # Full task: x, y, theta
    xdot = np.array([0.05, 0.0, 0.0])

    q1 = 0.0
    q3 = 0.0
    q2_vals = np.linspace(0.6, 0.01, 150)

    qdot_norms = []
    lambdas = []
    sigmas = []

    for q2 in q2_vals:
        q = np.array([q1, q2, q3])

        qdot, lam, sigma_min = ik_adaptive_dls(
            q, xdot, l,
            sigma_th=0.05,
            lam_max=0.1
        )

        qdot_norms.append(np.linalg.norm(qdot))
        lambdas.append(lam)
        sigmas.append(sigma_min)

    # =========================
    # PLOTS
    # =========================
    plt.figure(figsize=(10, 4))

    plt.subplot(1, 2, 1)
    plt.plot(q2_vals, qdot_norms)
    plt.xlabel("q2 → 0")
    plt.ylabel("||q̇||")
    plt.title("Adaptive DLS: Joint velocity")
    plt.grid(True)

    plt.subplot(1, 2, 2)
    plt.plot(q2_vals, lambdas, label="λ(q)")
    plt.plot(q2_vals, sigmas, label="σ_min(J)")
    plt.xlabel("q2 → 0")
    plt.title("Adaptive damping behavior")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()
