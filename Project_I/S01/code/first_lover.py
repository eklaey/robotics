# Lover implementation - Braitenberg vehicle with parallel-coupled motors
from unifr_api_epuck import wrapper
import numpy as np

MY_IP = '192.168.2.205'  # change robot number
robot = wrapper.get_robot(MY_IP)

NORM_SPEED = 2
PROX_TH = 100

# Weights for weighted proximity calculation - can be tuned for different behaviors (comment/uncomment)
# a, b, c, d = 1, 1, 1, 1        # Equal weights for all sensors
# a, b, c, d = 2, 1, 1, 0.5     # More weight to front sensors...
# a, b, c, d = 0.5, 1, 1, 2     # More weight to rear sensors...
a, b, c, d = 1, 1.5, 2, 4

robot.init_sensors()
robot.calibrate_prox()  

prox_array_right = []
prox_array_left = []

#---------------------------------------------
def equals(x, y):
    threshold = 1
    if x - y < threshold:
        return True
    else:
        return False   
#--------------------------------------------

# infinite loop
while robot.go_on():
    prox_values = robot.get_calibrate_prox()
    
    # Calculate weighted average for right and left sensors
    prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / (a + b + c + d)
    prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / (a + b + c + d)
    
    avg_prox_right = np.mean(prox_array_right[-10:]) if len(prox_array_right) >= 10 else prox_right
    avg_prox_left = np.mean(prox_array_left[-10:]) if len(prox_array_left) >= 10 else prox_left
        
    if equals(avg_prox_right, avg_prox_left) and avg_prox_left > PROX_TH:
        robot.set_speed(0)
        print("---Reached equilibrium at prox:", prox_left, prox_right)
        break
    else:
        print("Current prox:", prox_left, prox_right)
        # Calculate delta speeds using Braitenberg formula: ds = -(prox/prox_th) * s0
        ds_left = (NORM_SPEED * prox_left) / PROX_TH    # Left sensor controls left motor (parallel-coupled)
        ds_right = (NORM_SPEED * prox_right) / PROX_TH  # Right sensor controls right motor (parallel-coupled)
        
        # Calculate individual motor speeds: s = s0 + ds
        left_speed = NORM_SPEED - ds_left
        right_speed = NORM_SPEED - ds_right
        
        robot.set_speed(left_speed, right_speed)
        prox_array_right.append(prox_right)
        prox_array_right.pop(0)
        prox_array_left.append(prox_left)
        prox_array_left.pop(0)
    
robot.clean_up()
