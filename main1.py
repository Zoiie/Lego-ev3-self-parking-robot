#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.ev3devices import UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Initialize the EV3 Brick.
ev3 = EV3Brick()


# Initialize the motors.
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
font_motor = Motor(Port.B)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Initialize the Ultrasonic Sensor. It is used to detect
# obstacles as the robot drives around.
# print(UltrasonicSensor(Port.S1))
obstacle_sensor_right = UltrasonicSensor(Port.S1)
obstacle_sensor_font = UltrasonicSensor(Port.S3)
obstacle_sensor_back = UltrasonicSensor(Port.S2)

# Play a sound to tell us when we are ready to start moving
ev3.speaker.beep()

# The following loop makes the robot drive forward until it detects an
# obstacle. Then it backs up and turns around. It keeps on doing this
# until you stop the program.

# Begin driving forward at 200 millimeters per second.
# robot.drive(100, 0)

# Wait until an obstacle is detected. This is done by repeatedly
# doing nothing (waiting for 10 milliseconds) while the measured
# distance is still greater than 300 mm.
# while obstacle_sensor_font.distance() > 100:
#     wait(10)
#
# # Drive backward for 300 millimeters.
# robot.straight(-100)
#
# # Turn around by 120 degrees
# robot.turn(45)
dis_state_start = 0
dis_state_end = 0
begin_space=0
end_space=0
forward=0
dis_obstacle=0
dis_back=0
dis2_back=0
space=0
dis3_back=0

begin = True
begin_space_flag=False
end_space_flag=False

### Check the side obstacles
robot.drive(50, 0)

while begin==True:

    # font_motor.track_target(0)
    if obstacle_sensor_right.distance()<100:
        begin_space=robot.state()[0]
        begin_space_flag=True
        # print("begin_space_flag:",begin_space_flag)

    if begin_space_flag==True and obstacle_sensor_right.distance() >100:
        end_space=robot.state()[0]
        # print("end_space:",end_space)
        space=end_space - begin_space
        print("Space=",space)
    # print("obstacle_sensor_right.distance()=",obstacle_sensor_right.distance())
    # print("end_space-begin_space=",end_space-begin_space)

    if end_space-begin_space>70 and obstacle_sensor_right.distance()<100:
        end_space_flag=True
        # print("end_space_flag:",end_space_flag)
        dis_state_start = robot.state()[0]
        dis_obstacle=obstacle_sensor_right.distance()

    if obstacle_sensor_font.distance()<200:
        robot.stop()
        # if obstacle_sensor_font.distance()>200:


    while end_space_flag==True:
        robot.drive(50, 0)
        forward=robot.state()[0]

        while forward-dis_state_start>25 and space<130:
            robot.drive(-50,0)
            font_motor.track_target(-45)
            print(dis_state_start,robot.state()[0])
            print("dis_state_start-robot.state()[0]::",dis_state_start-robot.state()[0])
            dis_back=robot.state()[0]

            while dis_state_start-robot.state()[0]>170:
                font_motor.track_target(0)
                robot.drive(-50,0)
                print(robot.state()[0],dis_back,"robot.state()[0]-dis_back=",robot.state()[0]-dis_back)
                while robot.state()[0]-dis_back<-dis_obstacle:
                    # print("????")
                    robot.stop()

        while forward-dis_state_start>25 and space>=130:
            robot.drive(-50,0)
            font_motor.track_target(-45)
            print(dis_state_start,robot.state()[0])
            print("dis_state_start-robot.state()[0]::",dis_state_start-robot.state()[0])
            dis_back=robot.state()[0]

            while dis_state_start-robot.state()[0]>130:
                font_motor.track_target(45)
                robot.drive(-50,0)
                print(robot.state()[0],dis_back,"robot.state()[0]-dis_back=",robot.state()[0]-dis_back)
                dis2_back=robot.state()[0]
                print('robot.state()[0]-dis_back:',robot.state()[0]-dis_back)
                dis3_back=robot.state()[0]-dis_back
                print("obstacle_sensor_back.distance()=",obstacle_sensor_back.distance())

                while dis3_back<-120 or obstacle_sensor_back.distance()<20:
                    dis3_back=-121
                    font_motor.track_target(0)
                    robot.drive(50,0)
                    print("robot.state():",robot.state())
                    print(robot.state()[0]-dis2_back)
                    while robot.state()[0]-dis2_back>30:
                        robot.stop()
                    while obstacle_sensor_font.distance()<20:
                        robot.stop()

# else:
#     robot.drive(100,0)


