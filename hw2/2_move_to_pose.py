import numpy as np
import matplotlib.pyplot as plt


DEBUG = True

def r2d(x):
    return np.rad2deg(x)


def d2r(x):
    return np.deg2rad(x)


def goToPose(initial, goal):
    """
        initial: (x,y,a)
            Initial pose in XY coordinate
        goal: (x,y,a)
            Final pose in XY coordinate
    """
    Kr = 3
    Kb = -1.5
    Ka = 8
    dt = 0.01

    # Angle between robot direction and goal pose
    x,y,a = initial
    xg, yg, ag = goal
    
    robot_poses = [(x,y,a)]
    
    while True:
        rho = ((yg-y)*2 + (xg-x)**2)**0.5

        if rho <= 0.1 or len(robot_poses) >= 100:
            break 

        theta = ag - a
        alpha = -theta + np.arctan2((yg-y), (xg-x))
        beta = -theta -alpha

        # alpha and beta have to be between -pi and pi
        v = Kr * rho
        o = Ka * alpha  + Kb * beta

        x += v * np.cos(a) * dt
        y += v * np.sin(a) * dt
        a += o * dt

        robot_poses.append((x, y, a))

        if DEBUG:
            print(f'v: {v:>7.2f}| o: {o:>7.2f} deg/s | x: {x:>7.2f} | {y:>7.2f} | a: {a:>7.3f} deg |a_goal: {ag:7.2f}')


    return robot_poses


def plot_robot_poses(robot_poses, goal_point, scale=2):
    plt.figure(figsize=(10,10))
    robot_poses = np.array(robot_poses)  # N x 3
    init = robot_poses[0]
    x = robot_poses[:, 0]
    y = robot_poses[:, 1]
    a = robot_poses[:, 2]
    
    plt.plot(x,y, linestyle='--', marker='o', color='blue', markersize=scale*2, alpha=0.5,
        label='path')

    plt.scatter(init[0], init[1], c='black', marker='D', s=scale*50, label='init location')
    plt.scatter(goal_point[0], goal_point[1], c='red', marker='s', s=scale*50, label='goal')

    plt.arrow(init[0], init[1], scale*np.cos(d2r(init[2])), scale*np.sin(d2r(init[2])), 
        color='green', head_width=scale*0.6, length_includes_head=False, linestyle='-',
        label='init angle')

    plt.legend()
    plt.title(f'Init Pose: {init} | Goal location: {goal_point}')
    plt.show()
    plt.close()


if __name__ == '__main__':
    goal_pose = [-50,-50,np.pi/2]
    path = goToPose([0,0,0], goal_pose)

    plot_robot_poses(path, goal_pose)



