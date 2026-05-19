### imports

from unifr_api_epuck import wrapper
import cv2
from Project_II.vision import ArUcoCamera
import numpy as np
import math
import matplotlib.pyplot as plt
import os
import time
import sys


###################### Robot setup #####################################
MY_IP = '192.168.2.208' 
''
r = wrapper.get_robot(MY_IP)


############# initialize tracking (camera streaming) ###################
rtsp_url = f"rtsp://192.168.2.150:8554/cam2"
print(f"Connecting to {rtsp_url}...")
try:
    camera = ArUcoCamera(rtsp_url, marker_size_mm=40)
except Exception as e:
    print(f"Error initializing tracking stream: {e}")
    sys.exit(1)

R_wc = np.array([[-1,0,0],[0,1,0],[0,0,-1]])
C_w = np.array([[0],[0],[1.1]])

#camera infos
tx = None
ty = None
yaw = None

# last pose
last_tx, last_ty, last_yaw = None, None, None


################# variables ################
MARKER_ID = 0

startup_done = False
frame = None
markers = None

state = "EXPLORE"


################# functions ################
def run_exploration_logic(prox):
    # Thresholds
    SENS_WALL = 300 
    
    # Check if we are near a wall to follow
    if prox[2] > SENS_WALL or prox[1] > SENS_WALL: 
        # PID WALL FOLLOWER (Logic from your previous labs)
        error = target_distance - prox[2]
        correction = kp * error + kd * (error - last_error)
        return BASE_SPEED - correction, BASE_SPEED + correction
    
    else:
        # BRAITENBERG EXPLORER (Avoidance logic)
        # Weights for front sensors to turn away from obstacles
        speed_l = BASE_SPEED - (prox[0] * 0.1)
        speed_r = BASE_SPEED - (prox[7] * 0.1)
        return speed_l, speed_r

# --- Put this above your while loop ---
# Explorer Weights
ea, eb, ec, ed = 1, 1.5, 2, -2  #
EXPLORER_NORM = 1.5
EXPLORER_TH = 125 # (250/2)

# PID Weights
pa, pb, pc, pd = 4, 2, 1, 0  #
PID_NORM = 2.0
PID_TARGET = 200 #
PID_MAX_DS = 1.5 #

def get_phase1_speeds(prox, pid_controller):
    # 1. Calculate the weighted side proximity for the PID
    proxR_pid = (pa * prox[0] + pb * prox[1] + pc * prox[2] + pd * prox[3]) / (pa+pb+pc+pd) #
    
    # 2. TRIGGER CONDITION: Is there a wall on the right?
    # If the right side sensors see something significant, start wall following
    if prox[1] > 100 or prox[2] > 100: 
        
        # --- WALL FOLLOWER LOGIC ---
        ds = pid_controller.compute(proxR_pid, PID_TARGET) #
        ds += 0.05 # Turn towards wall by default
        
        speedR = PID_NORM + ds
        speedL = PID_NORM - ds
        
        # Clamping for corners
        if abs(ds) > PID_MAX_DS:
            speedR = ds
            speedL = -ds
            
        return speedL, speedR, "WALL_FOLLOWING"
        
    else:
        # --- EXPLORER LOGIC ---
        # Calculate weighted average for Braitenberg
        prox_right = (ea * prox[0] + eb * prox[1] + ec * prox[2] + ed * prox[3]) / (ea + eb + ec + ed)
        prox_left = (ea * prox[7] + eb * prox[6] + ec * prox[5] + ed * prox[4]) / (ea + eb + ec + ed)
        
        # Cross-coupled calculation
        ds_left = (EXPLORER_NORM * prox_right) / EXPLORER_TH   
        ds_right = (EXPLORER_NORM * prox_left) / EXPLORER_TH   
        
        speedL = EXPLORER_NORM - ds_left
        speedR = EXPLORER_NORM - ds_right
        
        return speedL, speedR, "EXPLORING"

################# main ################
while r.go_on():

    ### little timeout to ensure the camera stream is properly initialized before starting the main loop
    if not startup_done:
        start_time = time.time()
        while time.time() - start_time < 8.0:
            frame, markers = camera.get_marker_positions()  # flush RTSP frames
            if frame is None:
                print("Failed to get frame.")
                time.sleep(0.05)
                continue
            if markers is None:
                print("no marker :(")
            r.set_speed(0, 0)  # robot stays still

        startup_done = True
        continue
    

    ######### MAIN LOOP #########

    frame, markers = camera.get_marker_positions()
    prox = r.get_proximity_sensors()
    
    if state == "EXPLORE":
        # Implementation of Phase 1
        speed_l, speed_r = run_exploration_logic(prox)
        
        # Check transition to Phase 2
        if both_goals_found:
            state = "PLAN"

    elif state == "PLAN":
        # Implementation of Phase 2
        path = compute_path(my_map)
        state = "NAVIGATE"

    elif state == "NAVIGATE":
        # Implementation of Phase 3
        speed_l, speed_r = run_path_following(tx, ty, yaw, path)

    r.set_speed(speed_l, speed_r)
       
    cv2.imshow('RTSP Camera Stream', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


camera.release()
cv2.destroyAllWindows()
r.clean_up()