#!/usr/bin/env pybricks-micropython
#!/usr/bin/env python3

import numpy as np
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import parking

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# simulation parameters
Kp_rho = 9
Kp_alpha = 15
Kp_beta = -3
dt = 0.05

def check_obstacle(robot,dis,angle):

    # Initialize the Ultrasonic Sensor. It is used to detect
    # obstacles as the robot drives around.
    obstacle_sensor = UltrasonicSensor(Port.S4)

    # Play a sound to tell us when we are ready to start moving
    ev3.speaker.beep()

    # The following loop makes the robot drive forward until it detects an
    # obstacle. Then it backs up and turns around. It keeps on doing this
    # until you stop the program.

    # Begin driving forward at 200 millimeters per second.
    robot.drive(dis, angle)

    # Wait until an obstacle is detected. This is done by repeatedly
    # doing nothing (waiting for 10 milliseconds) while the measured
    # distance is still greater than 300 mm.
    while obstacle_sensor.distance() > 300:
        wait(10)

    # Drive backward for 300 millimeters.
    robot.straight(-dis)

    # Turn around by 120 degrees
    robot.turn(120)

def move_to_loc(start, theta_start, goal, theta_goal):
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

    return (x_traj,y_traj,theta_traj)

# def main():
# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
goal_loc = [20,20]
goal_ang = 0
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
(Distance, drive_speed, angle, turn_rate) = robot.state()
(x_traj, y_traj, angle_traj) = move_to_loc([0, 0], angle, goal_loc, goal_ang)
for i in range(0,len(x_traj)-1):
    dis=np.sqrt(((x_traj[i+1]-x_traj[i])**2+(y_traj[i+1]-y_traj[i])**2))
    angle_step=angle_traj[i+1]
    check_obstacle(robot,dis,angle_step)
