import numpy as np
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import parking

# Initialize the EV3 Brick.
ev3 = EV3Brick()

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

# def main():
# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Need to set points to initialize the goal location and goal angle
goal_loc = [20,20]
goal_ang = 0
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
[Distance, drive_speed, angle, turn_rate] = robot.state()
# Get trajectory
(x_traj, y_traj, angle_traj) = parking.move_to_pose([0, 0], angle, goal_loc, goal_ang)
for i in range(0,len(x_traj)-1):
    dis=np.sqrt(((x_traj[i+1]-x_traj[i])**2+(y_traj[i+1]-y_traj[i])**2))
    angle_step=angle_traj[i+1]
    check_obstacle(robot,dis,angle_step)