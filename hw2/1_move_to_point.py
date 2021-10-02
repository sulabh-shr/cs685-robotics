import numpy as np
import matplotlib.pyplot as plt


DEBUG = False


def r2d(x):
    return np.rad2deg(x)


def d2r(x):
    return np.deg2rad(x)

def positive_angle(x):
    if x < 0:
        return 360 + x
    return x

def goToPoint(initial_pose, goal_point, thresh=0.1):
    
    # Co-efficients & time parameter
    Kv = 0.1
    Kh = 0.3
    dt = 0.5

    x, y, a = initial_pose      # degrees
    x_goal, y_goal = goal_point

    robot_poses = [(x,y,a)]

    while (((x_goal-x)**2 + (y_goal - y)**2)**0.5) > thresh and len(robot_poses) <= 1000:
        # Get new difference in angle
        a_goal = r2d(np.arctan2((y_goal-y),(x_goal-x)))      # degrees
        # If angle is negative, get its positive equivalent
        a_goal = a_goal
        
        # Get new velocities
        v = Kv * ((x_goal-x)**2 + (y_goal - y)**2)**0.5
        o = Kh * (a_goal-a)
        o = o
        
        # Update current location
        x += v * np.cos(d2r(a)) * dt
        y += v * np.sin(d2r(a)) * dt
        a += o * dt
        
        if DEBUG:
            print(f'v: {v:>7.2f}| o: {o:>7.2f} deg/s | x: {x:>7.2f} | {y:>7.2f} | a: {a:>7.3f} deg |a_goal: {a_goal:7.2f}')

        # Save robot pose
        robot_poses.append((x, y, a))

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
    np.random.seed(1)
    scale = 100

    for i in range(5):
        init = (np.random.randint(-scale, scale), np.random.randint(-scale,scale), np.random.randint(0, 360))
        goal = (init[0] + np.random.randn()*scale//2, init[1]+np.random.randn()*scale//2)

        print(f'init = {init}')
        print(f'goal = {goal}')

        pred_robot_poses = goToPoint(init, goal)    
        plot_robot_poses(pred_robot_poses, goal)





    
