#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.nxtdevices import UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Initialize the EV3 Brick.
ev3 = EV3Brick()
flag_side=True
flag_side_start=True
flag_side_end=True
stop=False

# Initialize the motors.
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Initialize the Ultrasonic Sensor. It is used to detect
# obstacles as the robot drives around.
obstacle_sensor_right = UltrasonicSensor(Port.S4)
obstacle_sensor_font = UltrasonicSensor(Port.S3)
obstacle_sensor_back = UltrasonicSensor(Port.S2)

# Play a sound to tell us when we are ready to start moving
ev3.speaker.beep()

# Begin driving forward at 100 millimeters per second.
robot.drive(100, 0)

dis_state_start=0
dis_state_end=0

### Check the side obstacles
while obstacle_sensor_right.distance()>10 and obstacle_sensor_font.distance()>1000:
    flag_side=False
# If there are obstacles
while flag_side==False and obstacle_sensor_right.distance()>300:
    flag_side_start=False
    state_freelist = robot.state()  # Distance, drive speed, angle, turn rate
    dis_state_start=state_freelist[0]
while flag_side_start==False and obstacle_sensor_right.distance()>10:
    state_freelist2=robot.state()
    dis_state_end=state_freelist2[0]
    flag_side_end=False

if dis_state_end-dis_state_start>400:
    robot.drive(-100,45)
    while obstacle_sensor_back.distance()>10 and robot.state()[2]==90:
        robot.drive(-100,0)

else:
    robot.drive(100,0)

robot.stop()


### Check

if flag_side==False and flag_side_turn==False:
    robot.drive(100,45)
    state_list=robot.state() #Distance, drive speed, angle, turn rate
    ## Check if it's possible to park
    while state_list[2]==90 and obstacle_sensor_font.distance()>100:
        robot.drive(100,0)

    if state_list[2]-dis_state_freelist<500:
        robot.straight(-(state_list[2]-dis_state_freelist))




