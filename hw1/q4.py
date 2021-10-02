import numpy as np

a = 0.1729
b = -0.1468
c = 0.9739

R = np.array([
            [a, b, c],
            [c, a, b],
            [b, c, a]])
w, v = np.linalg.eig(R)

print(R)
print()
print(f'The eigen values and vectors are:')
for wi, vi in zip(w, v.T):
    print(f'Eigen value: {wi} | Eigen vector: {vi}')