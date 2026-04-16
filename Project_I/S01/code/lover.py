# Lover implementation - Braitenberg vehicle with parallel-coupled motors
from unifr_api_epuck import wrapper

MY_IP = '192.168.2.205'  # change robot number
robot = wrapper.get_robot(MY_IP)

NORM_SPEED = 1.5
PROX_TH = 250
PROX_TH /= 2  # Reduce threshold to make it more reactive to obstacles

# Weights for weighted proximity calculation - can be tuned for different behaviors (comment/uncomment)
# a, b, c, d = 1, 1, 1, 1   # Equal weights for all sensors
a, b, c, d = 1, 1.5, 2, 4   # More weight to side and rear sensors (sharper rotation towards loved objects)

robot.init_sensors()
robot.calibrate_prox()  

# infinite loop
while robot.go_on():
    prox_values = robot.get_calibrate_prox()
    
    # Calculate weighted average for right and left sensors
    prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / (a + b + c + d)
    prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / (a + b + c + d)
    
    # Calculate delta speeds using Braitenberg formula: ds = -(prox/prox_th) * s0
    ds_left = (NORM_SPEED * prox_left) / PROX_TH    # Left sensor controls left motor (parallel-coupled)
    ds_right = (NORM_SPEED * prox_right) / PROX_TH  # Right sensor controls right motor (parallel-coupled)
    
    # Calculate individual motor speeds: s = s0 + ds
    left_speed = NORM_SPEED - ds_left
    right_speed = NORM_SPEED - ds_right
    
    robot.set_speed(left_speed, right_speed)
    
robot.clean_up()