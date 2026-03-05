# Love at third time: AFSM that switches between LOVER and EXPLORER behaviors 3 times before settling in equilibrium
from unifr_api_epuck import wrapper
import time
import numpy as np

MY_IP = '192.168.2.205'  # change robot number
robot = wrapper.get_robot(MY_IP)    

NORM_SPEED = 1.5
PROX_TH = 250
PROX_TH /= 4  # Reduce threshold to make it more reactive to obstacles

# Weights for weighted proximity calculation (taken from lover.py and explorer.py from testing)
a, b, c, d = 1, 1.5, 2, 4

# State definitions
LOVER_STATE = "LOVER"
EXPLORER_STATE = "EXPLORER"
EQUILIBRIUM_STATE = "EQUILIBRIUM"

# Thresholds for robust condition detection (might need to be tuned based on robot behavior)
EQUILIBRIUM_SPEED_THRESHOLD = 1     # Robot almost stopped
EQUILIBRIUM_TIME_THRESHOLD = 1      # Must be stopped 
DEPARTURE_PROX_THRESHOLD = 100      # Far enough from obstacle
DEPARTURE_TIME_THRESHOLD = 1        # Must be away

# Initialize robot
robot.init_sensors()
robot.calibrate_prox()

prox_array = []  # To store recent proximity values for smoothing

# State machine variables
current_state = LOVER_STATE
obstacle_count = 0
state_entry_time = time.time()
timer_equilibrium_start = None
timer_departure_start = None

# Main loop
while robot.go_on():
    current_time = time.time()
    prox_values = robot.get_calibrate_prox()
    
    # Calculate weighted averages for right and left sensors
    prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / (a + b + c + d)
    prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / (a + b + c + d)
    
    # Calculate average proximity for equilibrium detection
    prox_avg = (prox_right + prox_left) / 2
    
    ##prox_array.append(prox_avg)
    ##if len(prox_array) > 10:
    ##    prox_array.pop(0)

    if current_state == LOVER_STATE:
        # LOVER: Parallel-coupling ==> attraction
        ds_left = (NORM_SPEED * prox_left) / PROX_TH
        ds_right = (NORM_SPEED * prox_right) / PROX_TH
        
        left_speed = NORM_SPEED - ds_left
        right_speed = NORM_SPEED - ds_right
        
        robot.set_speed(left_speed, right_speed)

        prev_prox_avg = np.mean(prox_array[-10:]) if len(prox_array) >= 10 else prox_avg

        
        # Detect equilibrium: robot almost stopped and in front of obstacle
        if prox_avg > PROX_TH and (prox_avg - prev_prox_avg) < 1:  # Check if proximity is high and stable
            if timer_equilibrium_start is None:
                timer_equilibrium_start = current_time
            
            # If stayed at low speed for enough time, reached equilibrium
            if current_time - timer_equilibrium_start >= EQUILIBRIUM_TIME_THRESHOLD:
                obstacle_count += 1
                print(f"Equilibrium reached! Obstacle #{obstacle_count}")
                
                if obstacle_count >= 3:
                    current_state = EQUILIBRIUM_STATE
                    print("Third obstacle! Staying in equilibrium.")
                    robot.enable_all_led()  # Keep LED on to indicate final state
                else:
                    current_state = EXPLORER_STATE
                    print("Switching to EXPLORER mode.")
                
                state_entry_time = current_time
                timer_equilibrium_start = None
                timer_departure_start = None
        else:
            timer_equilibrium_start = None
            
    elif current_state == EXPLORER_STATE:
        # EXPLORER: Cross-coupling ==> avoidance
        ds_left = (NORM_SPEED * prox_right) / PROX_TH
        ds_right = (NORM_SPEED * prox_left) / PROX_TH
        
        left_speed = NORM_SPEED - ds_left
        right_speed = NORM_SPEED - ds_right
        
        robot.set_speed(left_speed, right_speed)
        
        # Detect departure from obstacle: left and right proximity sensors low
        if prox_left < DEPARTURE_PROX_THRESHOLD and prox_right < DEPARTURE_PROX_THRESHOLD:
            if timer_departure_start is None:
                timer_departure_start = current_time
            
            # If stayed away long enough, confirmed departure
            if current_time - timer_departure_start >= DEPARTURE_TIME_THRESHOLD:
                print("Departure detected! Switching back to LOVER mode.")
                current_state = LOVER_STATE
                state_entry_time = current_time
                timer_equilibrium_start = None
                timer_departure_start = None
        else:
            timer_departure_start = None
            
    elif current_state == EQUILIBRIUM_STATE:
        # EQUILIBRIUM: Robot stays in place facing the third obstacle ==> no movement
        robot.set_speed(0, 0)
        print("Staying in equilibrium facing the obstacle. FINISHED!")

robot.clean_up()
