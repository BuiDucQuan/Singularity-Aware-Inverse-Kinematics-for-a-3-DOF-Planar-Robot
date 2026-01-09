import numpy as np

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

