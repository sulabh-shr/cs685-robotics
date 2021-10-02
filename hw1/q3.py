import numpy as np


theta = np.radians(30)
phi = np.radians(45)
P = np.array([
                [2],
                [0],
                [3]])
rz = np.array([
                [np.cos(theta), -np.sin(theta), 0], 
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1]])
rx = np.array([
                [1, 0, 0],
                [0, np.cos(phi), -np.sin(phi)],
                [0, np.sin(phi), np.cos(phi)]])
R = np.matmul(rx, rz)
Rinv = np.linalg.inv(R)

Q = np.matmul(Rinv, P)
print(f'The coordinate of P in the frame A\' is:' )
print(Q)
