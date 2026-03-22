# run the code to generate IR sensor data 
from unifr_api_epuck import wrapper
import numpy as np

MY_IP = '192.168.2.208'
MAX_STEPS = 2000

# Sensor definition (in array returned by robot.get_ground())
G_LEFT = 0
G_MIDDLE = 1
G_RIGHT = 2

# Threshold to determine wether a sensor is on or off the line
G_THRESHOLD = 700

# Check if sensor is on the line
def on_line(gs):
    if gs < G_THRESHOLD: # sensor is on the line
        return 1
    else:
        return 0


# Initial conditions for state machine
SPEED = 1.5

# Initialize robot
robot = wrapper.get_robot(MY_IP)
robot.init_ground()
robot.sleep(5)


for step in range(MAX_STEPS):
    robot.go_on()
    gs = robot.get_ground()

    match [on_line(gs[G_LEFT]), on_line(gs[G_MIDDLE]), on_line(gs[G_RIGHT])]:
        case [1, 1, 0]: # correct, go straight
            print("1 1 0 -> straight")
            speed_left, speed_right = SPEED, SPEED
        case [1, 0, 0]: # must turn left
            print("1 0 0 -> left")
            speed_left, speed_right = 0, SPEED
        case [1, 1, 1]: # must turn right
            print("1 1 1 -> right")
            speed_left, speed_right = SPEED, 0
        case [0, 1, 1] | [0, 0, 1]: # must turn right
            print("0 * 1 -> right")
            speed_left, speed_right = SPEED, 0
        case _: # by default, go left
            print("default -> left")
            speed_left, speed_right = 0, SPEED

    robot.set_speed(speed_left, speed_right)
    

robot.clean_up()
