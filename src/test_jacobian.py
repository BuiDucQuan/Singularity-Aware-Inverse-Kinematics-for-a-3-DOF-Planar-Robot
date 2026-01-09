import numpy as np

def fk(q, l=[1.0, 1.0, 1.0]):
    q1, q2, q3 = q
    l1, l2, l3 = l

    x = l1*np.cos(q1) + l2*np.cos(q1+q2) + l3*np.cos(q1+q2+q3)
    y = l1*np.sin(q1) + l2*np.sin(q1+q2) + l3*np.sin(q1+q2+q3)
    return np.array([x, y])

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

def jacobian_fd(q, eps=1e-6):
    J = np.zeros((2, 3))
    x0 = fk(q)

    for i in range(3):
        dq = np.zeros(3)
        dq[i] = eps
        J[:, i] = (fk(q + dq) - x0) / eps

    return J

# ===================== TEST =====================
q_test = np.array([0.4, -0.6, 0.3])

J_analytic = jacobian(q_test)
J_fd = jacobian_fd(q_test)

print("Jacobian analytic:\n", J_analytic)
print("\nJacobian finite diff:\n", J_fd)
print("\nError norm:", np.linalg.norm(J_analytic - J_fd))
