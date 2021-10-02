import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt


def plot_3d(x, y, z, title=None):
    fig = plt.figure(figsize=(25, 25))
    ax = fig.add_subplot(projection='3d')
    ax.plot_surface(x, y, z)
    plt.title(title)
    plt.show()
    plt.close()


def dist_to_point(map_x, map_y, point):
    point_x, point_y = point
    dist_x = map_x - point_x
    dist_y = map_y - point_y
    dist = np.sqrt(np.power(dist_x, 2) + np.power(dist_y, 2))

    return dist


def attractive_potential(map_x, map_y, goal):
    goal_x, goal_y = goal
    dist_x = map_x - goal_x
    dist_y = map_y - goal_y
    dist = np.sqrt(np.power(dist_x, 2) + np.power(dist_y, 2))

    return dist


def attractive_potential_grad(map_x, map_y, goal):
    goal_x, goal_y = goal
    dist_x = map_x - goal_x
    dist_y = map_y - goal_y

    return -dist_x, -dist_y


def repulsive_potential(map_x, map_y, obstacles, radius):
    obstacle_distances = np.zeros((len(obstacles), map_x.shape[0], map_x.shape[1]))

    for idx, obstacle in enumerate(obstacles):
        obstacle_distances[idx] = dist_to_point(map_x, map_y, obstacle)
        # plt.contour(map_x, map_y, obstacle_distances)

    obstacle_distances = np.min(obstacle_distances, axis=0)
    obstacle_distances = np.clip(obstacle_distances, a_min=1e-6, a_max=None)

    # plot_3d(map_x, map_y, obstacle_distances, 'Original Obstacle Distance')

    radius_mask = obstacle_distances <= radius
    potentials = np.zeros_like(obstacle_distances)
    potentials[radius_mask] = np.power((1 / obstacle_distances[radius_mask]) - (1 / radius), 2)
    # plot_3d(map_x, map_y, potentials, 'Repulsive potential')

    return potentials


def repulsive_potential_grad(map_x, map_y, obstacles, radius):
    obstacle_distances = np.zeros((len(obstacles), map_x.shape[0], map_x.shape[1]))

    for idx, obstacle in enumerate(obstacles):
        obstacle_distances[idx] = dist_to_point(map_x, map_y, obstacle)

    obstacle_distances = np.min(obstacle_distances, axis=0)
    obstacle_distances = np.clip(obstacle_distances, a_min=1e-6, a_max=None)
    radius_mask = obstacle_distances <= radius
    potentials = np.zeros_like(obstacle_distances)
    potentials[radius_mask] = ((1 / obstacle_distances[radius_mask]) - (1 / radius)) / obstacle_distances[radius_mask] ** 3

    grad_x = np.zeros_like(obstacle_distances)
    grad_y = np.zeros_like(obstacle_distances)

    grad_x[radius_mask] = potentials[radius_mask] * (map_x[radius_mask] - obstacle_distances[radius_mask])
    grad_y[radius_mask] = potentials[radius_mask] * (map_y[radius_mask] - obstacle_distances[radius_mask])

    # fig, ax = plt.subplots(figsize=(10, 10))
    # ax.set(xlim=(0, 100), ylim=(0, 100))
    # ax.streamplot(map_x, map_y, grad_x, grad_y, density=2)
    # plt.show()

    return grad_x, grad_y


if __name__ == '__main__':
    di = 0.1
    coeff_att = 1
    coeff_rep = 1
    goal = (30, 20)
    init = (x, y) = (50, 50)

    obstacles = ((40, 30), (70, 40))
    r = 5
    coords_x, coords_y = np.meshgrid(np.arange(100), np.arange(100))
    if True:
        f1 = attractive_potential(coords_x, coords_y, goal)
        plot_3d(coords_x, coords_y, f1, 'Attractive Potential Field')
        f2 = repulsive_potential(coords_x, coords_y, obstacles, radius=r)
        f = -(coeff_att * f1 + coeff_rep * f2)
        plot_3d(coords_x, coords_y, f, 'Potential field')
        grad_x, grad_y = np.gradient(f, edge_order=2)
    else:
        f1_x, f1_y = attractive_potential_grad(coords_x, coords_y, goal)
        f2_x, f2_y = repulsive_potential_grad(coords_x, coords_y, obstacles, radius=r)
        grad_x = coeff_att * f1_x + coeff_rep * f2_x
        grad_y = coeff_att * f1_y + coeff_rep * f2_y

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set(xlim=(0, 100), ylim=(0, 100))
    plt.scatter(init[0], init[1], c='black', marker='D', s=50, label='init location')
    plt.scatter(goal[0], goal[1], c='red', marker='s', s=50, label='goal')
    for obstacle in obstacles:
        ax.add_patch(plt.Circle(obstacle, r, fill=False))
    ax.streamplot(coords_x, coords_y, grad_x, grad_y, density=2, linewidth=1, arrowsize=1, maxlength=0.5)
    # plt.show()
    x_list = [x]
    y_list = [y]
    print(f'start')
    print(np.min(grad_x), np.min(grad_y))

    while (grad_x[x, y] > np.min(grad_x) or grad_y[x, y] > grad_y[x,y]) and len(x_list) < 20:
        dx = grad_x[x, y]
        dy = grad_y[x, y]
        dxi = int(np.around(di * dx))
        dyi = int(np.around(di * dy))
        x = x + dxi
        y = y + dyi
        x_list.append(x)
        y_list.append(y)
        print(x, y)
    plt.scatter(x_list, y_list)
    plt.legend()
    plt.show()
    plt.close()
