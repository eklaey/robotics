# run the code to generate IR sensor data 
from unifr_api_epuck import wrapper
import numpy as np

MY_IP = '192.168.2.208'
MAX_STEPS = 2000

# Behavior parameters for tuning robotic "love" and "exploration" dynamics
G_THRESHOLD = 700


# Sensor definition
G_LEFT = 0
G_MIDDLE = 1
G_RIGHT = 2

# Check if sensor is on the line
def on_line(gs):
    if gs < G_THRESHOLD: # sensor is on the line
        return 1
    else:
        return 0


# Initial conditions for state machine
SPEED = 1.5

# Setup for moving average
# https://en.wikipedia.org/wiki/Moving_average
k = 3
ground_array = []
ma_left_k = 0
ma_middle_k = 0
ma_right_k = 0 

# Initialize robot
# robot.init_sensors() 
robot = wrapper.get_robot(MY_IP)
robot.init_ground()
robot.sleep(5)



for step in range(MAX_STEPS):
    robot.go_on()
    gs = robot.get_ground()
    ground_array.append(gs)

    #no move for k steps
    n = step+1

    # initial setup for moving average, need k values
    if n==k:                                  
        ma_left_k   = np.mean(ground_array[G_LEFT][-k:])
        ma_middle_k = np.mean(ground_array[G_MIDDLE][-k:])
        ma_right_k  = np.mean(ground_array[G_RIGHT][-k:])
        print(ground_array)
        print(ma_left_k, ma_middle_k, ma_right_k)

    if n > k:
        ma_left_k   += 1/k*(gs[G_LEFT] - ground_array[n-k-1][G_LEFT])
        ma_middle_k += 1/k*(gs[G_MIDDLE] - ground_array[n-k-1][G_MIDDLE])
        ma_right_k  += 1/k*(gs[G_RIGHT] - ground_array[n-k-1][G_RIGHT])
        
        #print (on_line(gs[G_LEFT]), on_line(gs[G_MIDDLE]), on_line(gs[G_RIGHT]))
        print ( ma_left_k, ma_middle_k, ma_right_k)
        print (gs)

        #match [on_line(ma_left_k), on_line(ma_middle_k), on_line(ma_right_k)]: # using moving average
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
            case _: # by default, go straight
                print("default -> stop")
                speed_left, speed_right = 0, SPEED

        robot.set_speed(speed_left, speed_right)
    

robot.clean_up()
