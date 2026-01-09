import sys
import os
import numpy as np
import matplotlib.pyplot as plt



def manipulability(J):
    return np.sqrt(np.linalg.det(J @ J.T))


q2_vals = np.linspace(-np.pi, np.pi, 300)
w_vals = []

for q2 in q2_vals:
    J = jacobian([0.0, q2, 0.0])
    w_vals.append(manipulability(J))

w_vals = np.array(w_vals)

idx = np.argmin(w_vals)
q2_sing = q2_vals[idx]

print("Singular configuration:")
print(f"q = [0, {q2_sing:.4f}, 0]")
print(f"w = {w_vals[idx]:.3e}")

plt.plot(q2_vals, w_vals)
plt.axvline(q2_sing, linestyle="--", label="Singularity")
plt.xlabel("q2 (rad)")
plt.ylabel("Manipulability w")
plt.legend()
plt.grid()
plt.show()


