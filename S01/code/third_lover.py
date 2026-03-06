# Love at third time: AFSM that switches between LOVER and EXPLORER behaviors 3 times before settling in equilibrium
from unifr_api_epuck import wrapper
import numpy as np

MY_IP = '192.168.2.205'     # REMEMBER: change last IP number to ROBOT number
robot = wrapper.get_robot(MY_IP)    

# Behavior parameters for tuning robotic "love" and "exploration" dynamics
NORM_SPEED = 1.5 + 0.5      # Increase base speed to make behavior more dynamic
PROX_TH = 250 - 100         # Reduce threshold to make it more reactive to obstacles              
STATE_STEPS_TH = 300        # Minimum steps to stay in a state before switching (robust prevention of rapid oscillation)
EQUILIBRIUM_TH = 10         # Threshold for considering proximity values as "stable" for equilibrium detection
SMOOTHING_WINDOW_SIZE = 10  # Number of recent proximity values to average for smoothing

# Weights for weighted proximity calculation (taken from lover.py and explorer.py from testing)
a, b, c, d, e = 1, 1.5, 2, 4, -2 #d->lover, e->explorer

# State definitions
LOVER_STATE = "LOVER"
EXPLORER_STATE = "EXPLORER"
EQUILIBRIUM_STATE = "EQUILIBRIUM"

# Storage of recent proximity values for smoothing; will trim to last store only 10 values manually
prox_array = []

# Initial conditions for state machine
current_state = LOVER_STATE
obstacle_counter = 0
state_step_counter = 0

# Initialize robot
robot.init_sensors()
robot.calibrate_prox()
robot.sleep(5)          # give some time for experiment setup before starting main loop 

# MAIN LOOP --------------------------------------------------------------------------------------------------------------------
while robot.go_on():
    prox_values = robot.get_calibrate_prox()

    prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / (a + b + c + d)
    prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / (a + b + c + d)
    
    prox_avg = (prox_right + prox_left) / 2
    
    # Updating sliding window and trim to last 10 values
    prox_array.append(prox_avg)
    if len(prox_array) > SMOOTHING_WINDOW_SIZE:
        prox_array = prox_array[-SMOOTHING_WINDOW_SIZE:]
    prev_prox_avg = np.mean(prox_array)
    
    state_step_counter += 1  # Increment step counter for current state

    # AUGMENTED FINITE STATE MACHINE -------------------------------------------------------------------------------------------
    if current_state == LOVER_STATE:
        robot.enable_all_led()  # Turn on LED on to indicate LOVER state
        
        # LOVER: Parallel-coupling ==> attraction
        if not (prev_prox_avg > PROX_TH and 
                abs(prox_avg - prev_prox_avg) < EQUILIBRIUM_TH and
                state_step_counter > STATE_STEPS_TH):  
            ds_left = (NORM_SPEED * prox_left) / PROX_TH
            ds_right = (NORM_SPEED * prox_right) / PROX_TH
            
            left_speed = NORM_SPEED - ds_left
            right_speed = NORM_SPEED - ds_right
            
            robot.set_speed(left_speed, right_speed)
        else:
            robot.set_speed(0)
            
# potentially put state_step_counter comparison here to ensure it stayed in this state long enough before confirming love
            
            obstacle_counter += 1
            print(f"LOVE reached! Obstacle #{obstacle_counter} at prox: {prox_avg}")
                
            if obstacle_counter >= 3:
                current_state = EQUILIBRIUM_STATE
                print("EQUILIBRIUM reached!")
            else:
                current_state = EXPLORER_STATE
                print("EXPLORER activated!")
            
            state_step_counter = 0  # Reset step counter for new state
            
    elif current_state == EXPLORER_STATE:
        robot.disable_all_led()  # Turn off LED to indicate EXPLORER state
        
        # EXPLORER: Cross-coupling ==> avoidance
        if not (prev_prox_avg < PROX_TH and 
                abs(prox_avg - prev_prox_avg) < EQUILIBRIUM_TH and
                state_step_counter > STATE_STEPS_TH):
            ds_left = (NORM_SPEED * prox_right) / PROX_TH
            ds_right = (NORM_SPEED * prox_left) / PROX_TH
            
            left_speed = NORM_SPEED - ds_left
            right_speed = NORM_SPEED - ds_right
            
            robot.set_speed(left_speed, right_speed)
        else:
            robot.set_speed(0)
            current_state = LOVER_STATE
            print(f"DEPARTURE reached! SWITCHING back to LOVER at prox: {prox_avg}")
                        
            state_step_counter = 0  # Reset step counter for new state
            
    elif current_state == EQUILIBRIUM_STATE:
        # Toggle LED to indicate EQUILIBRIUM state
        if state_step_counter % 20 < 10:
            robot.disable_all_led()  
        else:
            robot.enable_all_led()  

        robot.set_speed(0)

        if state_step_counter > STATE_STEPS_TH:
            print(f"STABLE EQUILIBRIUM maintained at prox: {prox_avg}")
            break  # Exit loop and end program after confirming stable equilibrium
    # AUGMENTED FINITE STATE MACHINE (END) --------------------------------------------------------------------------------------

robot.clean_up()