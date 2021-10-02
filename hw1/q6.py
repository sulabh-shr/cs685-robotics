import numpy as np
import matplotlib.pyplot as plt


def diffDrive(init_pos, v, omega, t, delta):
    x=init_pos[0]
    y=init_pos[1]
    th=init_pos[2]

    x_list = []
    y_list = []
    th_list = []

    for i in range(1, t+1):
        x = x + delta * v * np.cos(np.deg2rad(th))
        y = y + delta * v * np.sin(np.deg2rad(th))
        th = th + delta * omega

        x_list.append(x)
        y_list.append(y)
        th_list.append(th)

    print(f'X = {np.round(x_list, 2)}')
    print(f'X = {np.round(y_list, 2)}')
    print(f'X = {np.round(th_list, 2)}')

    return x_list, y_list, th_list


if __name__ == '__main__':
    init_pos = [100, 50, 45]
    for dt in [1, 5, 10, 20, 25]:
        [xo, yo, to] = diffDrive(init_pos, v=1, omega=2, t=10, delta=dt)

            # plt.scatter([init_pos[0]] + x_list, [init_pos[1]] + y_list)
        plt.plot([init_pos[0]] + xo, [init_pos[1]] + yo, marker='s', linestyle='--', label=f'dt={dt}')
        plt.scatter(init_pos[0], init_pos[1], color='r', marker='s')

        plt.title(f'delta = {dt}')
    plt.legend()        
    plt.show()  
    plt.close()