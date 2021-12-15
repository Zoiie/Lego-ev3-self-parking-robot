
import matplotlib.pyplot as plt
import numpy as np

# simulation parameters
Kp_rho = 9
Kp_alpha = 15
Kp_beta = -3
dt = 0.05

show_animation = True


def move_to_pose(start, theta_start, goal, theta_goal):
    x = start[0]
    y = start[1]
    theta = theta_start

    x_diff = goal[0] - x
    y_diff = goal[1] - y

    x_traj, y_traj = [], []
    theta_traj=[]

    rho = np.hypot(x_diff, y_diff)
    while rho > 0.001:
        x_traj.append(x)
        y_traj.append(y)
        theta_traj.append(theta)

        x_diff = goal[0] - x
        y_diff = goal[1] - y

        # Restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

        v = Kp_rho * rho
        w = Kp_alpha * alpha + Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        theta = theta + w * dt
        x = x + v * np.cos(theta) * dt
        y = y + v * np.sin(theta) * dt

        print("Location:(",x,",",y,",","theta",theta,")")

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.arrow(start[0], start[1], np.cos(theta_start),
                      np.sin(theta_start), color='r', width=0.1)
            plt.arrow(goal[0], goal[1], np.cos(theta_goal),
                      np.sin(theta_goal), color='g', width=0.1)
            plot_vehicle(x, y, theta, x_traj, y_traj)

    return (x_traj,y_traj,theta_traj)


def plot_vehicle(x, y, theta, x_traj, y_traj):  # pragma: no cover
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    plt.plot(x_traj, y_traj, 'b--')

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(dt)


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

def get_point():
    plt.xlim(0,20)
    plt.ylim(0,20)
    position = plt.ginput(4)
    print(position)
    return position

def angle_of_vector(point, direction):
    # x1,y1 = v1
    # x2,y2 = v2
    # dot = x1*x2+y1*y2
    # det = x1*y2-y1*x2
    # theta = np.arctan2(det, dot)
    # theta = theta if theta>0 else 2*np.pi+theta
    # print("degree:",theta)
    # return np.deg2rad(theta)
    v1=[1,0]
    v2=[direction[0]-point[0],direction[1]-point[1]]


    # 2个向量模的乘积
    TheNorm = np.linalg.norm(v1) * np.linalg.norm(v2)
    # cross
    rho = np.rad2deg(np.arcsin(np.cross(v1, v2) / TheNorm))
    # dot
    theta = np.rad2deg(np.arccos(np.dot(v1, v2) / TheNorm))
    print(theta)
    if rho < 0:
        return np.deg2rad(- theta+360)
    else:
        return np.deg2rad(theta)
    # if rho < 0:
    #     return - theta+360
    # else:
    #     return theta

def main():
    #
    # for i in range(5):
    #     x_start = 20 * random()
    #     y_start = 20 * random()
    #     theta_start = 2 * np.pi * random() - np.pi
    #     x_goal = 20 * random()
    #     y_goal = 20 * random()
    #     theta_goal = 2 * np.pi * random() - np.pi
    #     print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
    #           (x_start, y_start, theta_start))
    #     print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
    #           (x_goal, y_goal, theta_goal))
    #     move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)
    # Get start point, goal point and their directions
    start,start_direction,goal,goal_direction=get_point()
    print(start,start_direction,goal,goal_direction)
    # calculate the angle
    theta_start=angle_of_vector(start,start_direction)
    theta_goal=angle_of_vector(goal,goal_direction)
    print("theta_start:",theta_start,"\n","theta_goal:",theta_goal)
    # planning
    move_to_pose(start,theta_start,goal,theta_goal)


if __name__ == '__main__':
    main()
