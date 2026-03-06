# Love at third time: AFSM that switches between LOVER and EXPLORER behaviors 3 times before settling in equilibrium
from unifr_api_epuck import wrapper
import numpy as np

MY_IP = '192.168.2.205'  # change robot number
robot = wrapper.get_robot(MY_IP)    

NORM_SPEED = 2
PROX_TH = 150
PROX_FAR_TH = 1
STEPS_TH = 300

# Weights for weighted proximity calculation (taken from lover.py and explorer.py from testing)
a, b, c, d, e = 1, 1.5, 2, 4, -2 #d->lover, e->explorer

# State definitions
LOVER_STATE = "LOVER"
EXPLORER_STATE = "EXPLORER"
EQUILIBRIUM_STATE = "EQUILIBRIUM"


# Initialize robot
robot.init_sensors()
robot.calibrate_prox()

prox_array_right = []
prox_array_left = []
prox_left=0
prox_right=0

#---------------------------------------------
def equals(x, y):
    threshold = 10
    if x - y < threshold:
        return True
    else:
        return False   
#--------------------------------------------

# State machine variables
current_state = LOVER_STATE
obstacle_count = 0
current_state_steps = 0

# Main loop
while robot.go_on():
    
    prox_values = robot.get_calibrate_prox()
    current_state_steps += 1

    avg_prox_right = np.mean(prox_array_right[-10:]) if len(prox_array_right) >= 10 else prox_right
    avg_prox_left = np.mean(prox_array_left[-10:]) if len(prox_array_left) >= 10 else prox_left
         
    if current_state == LOVER_STATE:
        robot.enable_all_led()  # Keep LED on to indicate lover state

        # Calculate weighted average for right and left sensors
        prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / (a + b + c + d)
        prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / (a + b + c + d)
        
        if equals(avg_prox_right, avg_prox_left) and avg_prox_left > PROX_TH and current_state_steps > STEPS_TH:
            robot.set_speed(0)
            print("---Found love number #{obstacle_count} at prox:", prox_left, prox_right)
           
            current_state_steps = 0
            obstacle_count += 1
                
            if obstacle_count >= 3:
                current_state = EQUILIBRIUM_STATE
                print("Third lover found! Staying in equilibrum.")
                
            else:
                current_state = EXPLORER_STATE
                print("Switching to EXPLORER mode.")
                
        
        else:
            print("Still searching for love ...")
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
            
    elif current_state == EXPLORER_STATE:
        robot.disable_all_led()  # Keep LED on to indicate lover state

        # Calculate weighted average for right and left sensors
        prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + e * prox_values[3]) / (a + b + c + d)
        prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + e * prox_values[4]) / (a + b + c + d)
        
           
        # EXPLORER: Cross-coupling ==> avoidance
        if (abs(avg_prox_left) + abs(avg_prox_right))/2 < PROX_FAR_TH and current_state_steps>STEPS_TH:
            robot.set_speed(0)
            print("---Done with exploring.")
           
            current_state_steps = 0
            current_state = LOVER_STATE
            print("Switching to LOVER mode.")
                
        
        else:
            print("Still exploring ...")
            # Calculate delta speeds using Braitenberg formula: ds = -(prox/prox_th) * s0
            ds_left = (NORM_SPEED * prox_right) / PROX_TH    # Left sensor controls left motor (parallel-coupled)
            ds_right = (NORM_SPEED * prox_left) / PROX_TH  # Right sensor controls right motor (parallel-coupled)
            
            # Calculate individual motor speeds: s = s0 + ds
            left_speed = NORM_SPEED - ds_left
            right_speed = NORM_SPEED - ds_right
            
            robot.set_speed(left_speed, right_speed)
            prox_array_right.append(prox_right)
            prox_array_right.pop(0)
            prox_array_left.append(prox_left)
            prox_array_left.pop(0)
        
        
            
    elif current_state == EQUILIBRIUM_STATE:
        if current_state_steps%20 < 10:
            robot.disable_all_led()  # Keep LED on to indicate lover state
        else:
            robot.enable_all_led()  # Keep LED on to indicate lover state

        # EQUILIBRIUM: Robot stays in place facing the third obstacle ==> no movement
        robot.set_speed(0, 0)
        print("Third love has been found.")
        if current_state_steps>STEPS_TH:
            print("Goodbye!")
            break

robot.clean_up()
