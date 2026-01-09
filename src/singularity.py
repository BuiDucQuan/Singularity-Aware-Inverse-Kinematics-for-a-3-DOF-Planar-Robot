import numpy as np
import matplotlib.pyplot as plt
from jacobian import jacobian

q2 = np.linspace(-np.pi, np.pi, 300)
sigma = []

for qi in q2:
    J = jacobian([0, qi, 0])
    sigma.append(np.linalg.svd(J, compute_uv=False)[-1])

sigma = np.array(sigma)
q2_sing = q2[np.argmin(sigma)]

print("q = [0, {:.4f}, 0]".format(q2_sing))
print("σ_min =", sigma.min())

plt.plot(q2, sigma)
plt.axvline(q2_sing, ls='--')
plt.xlabel("q2"); plt.ylabel("σ_min")
plt.grid(True); plt.show()



