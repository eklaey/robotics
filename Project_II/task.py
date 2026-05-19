from unifr_api_epuck import wrapper

from vision import ArUcoCamera
from threaded_camera import ThreadedCamera
from pid import PID

import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import sys

############# initialize tracking (camera streaming) ###################
rtsp_url = f"rtsp://192.168.2.150:8554/cam2"
print(f"Connecting to {rtsp_url}...")
try:
    camera = ArUcoCamera(rtsp_url, marker_size_mm=40)
except Exception as e:
    print(f"Error initializing tracking stream: {e}")
    sys.exit(1)
threaded_cam = ThreadedCamera(camera)

R_wc = np.array([[-1,0,0],[0,1,0],[0,0,-1]])
C_w = np.array([[0],[0],[1.1]])

#camera infos
tx = None
ty = None
yaw = None

frame = None
markers = None
last_tx, last_ty, last_yaw = None, None, None

frame_count = 0

###################### Robot setup #####################################
########################################################################
MY_IP = '192.168.2.202' # Robot IP to be change accordingly to the used one
r = wrapper.get_robot(MY_IP)

####################### variables ######################################

MARKER_ID = 10    # ArUco marker ID to be change accordingly to the used one

NORM_SPEED = 1.5

x_min = x_max = y_min = y_max = 0
resolution = 0.02 
path = []   # stores (x, y) robot positions 

startup_done = False

last_resync = 0
has_not_resync = True
resync_start_time = None
RESYNC_DURATION = 7.0
RECALIBRATION_TIMEOUT = 30

# Braitenberg EXPLORER
PROX_TH = 125
ea, eb, ec, ed = 1, 2, 2, -1

# PID Parameters for Wall Following
PID_MAX_DS = 1.5
PID_WALL_TARGET = 200
wa, wb, wc, wd = 4, 2, 1, 0

K = 0.005    
T_D = 0
T_I = 9999999999

# Instantiate the PID controller
wall_pid = PID(K, T_I, T_D)

# Behavior tracking variables
resync_interruption_start = None

wall_follow_start_time = None
WALL_FOLLOW_DURATION = 30.0  # Time in seconds to follow a single block before breaking away


###### simple state machine having basic and resync behaviors ##########
BASE = "BASE"
RE_SYNC = "RE_SYNC"

state = BASE

###### finite state machine for behavior switching in task #############
LOVER = "LOVER"
EXPLORER = "EXPLORER"
WALL_FOLLOWER = "WALL_FOLLOWER"
PATH_FINDER = "PATHFINDER"

mode = EXPLORER

###### functions ########
def world_to_grid(tx, ty):
    ix = int((tx - x_min) / resolution)
    iy = int((ty - y_min) / resolution)
    return ix, iy
def grid_to_world(ix, iy):
    wx = x_min + (ix + 0.5) * resolution
    wy = y_min + (iy + 0.5) * resolution
    return wx, wy
def update_pose():
    global last_tx, last_ty, last_yaw, frame, markers

    try:
        # new_frame, new_markers = camera.get_marker_positions()
        new_frame, new_markers = threaded_cam.get_marker_positions()
    except Exception as e:
        print("Camera read error:", e)
        return last_tx, last_ty, last_yaw

    if new_frame is None or not hasattr(new_frame, "size") or new_frame.size == 0:
        print("Invalid frame received")
        return last_tx, last_ty, last_yaw

    frame = new_frame
    markers = new_markers

    if not markers or MARKER_ID not in markers:
        return last_tx, last_ty, last_yaw

    try:
        tvec = np.array(markers[MARKER_ID]['tvec']).reshape(3, 1)
        t_w = R_wc @ tvec + C_w

        tx = t_w[0].item()
        ty = t_w[1].item()

        R_cm, _ = cv2.Rodrigues(markers[MARKER_ID]['rvec'])
        R_wm = R_wc @ R_cm
        yaw = math.atan2(R_wm[1, 0], R_wm[0, 0])

        last_tx, last_ty, last_yaw = tx, ty, yaw
    except Exception as e:
        print("Pose parsing error:", e)
    return last_tx, last_ty, last_yaw
def set_cell_if_empty(grid, x, y, value):
    if 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]:
        if grid[y, x] == 0:
            grid[y, x] = value
###################### MAP PREPARATION #####################################
print("Waiting for valid ArUco pose to initialize map...")
tx, ty, yaw = None, None, None
init_map_start_time = time.time()

# Keep trying to get a pose until successful (max 15 seconds)
while tx is None or ty is None:
    if time.time() - init_map_start_time > 15:
        print("Timeout: Could not find ArUco marker within 30 seconds. Terminating program.")
        sys.exit(1)

    tx, ty, yaw = update_pose()
    if tx is None:
        time.sleep(0.1) # Wait slightly for the camera thread to catch up

# Now that we have a real position, define borders
x_min = tx - 0.45
x_max = tx + 0.45
y_min = ty - 0.30
y_max = ty + 0.30

# create grid map
nx = int((x_max - x_min) / resolution)
ny = int((y_max - y_min) / resolution)
grid = np.zeros((ny, nx))
print(f"Map initialized successfully: x[{x_min:.2f}, {x_max:.2f}] y[{y_min:.2f}, {y_max:.2f}")

###################### SENSOR CALIBRATION ###################################
r.init_sensors()
r.calibrate_prox()

#############################################################################
######################## GO_ON LOOP #########################################
while r.go_on():

    ################## PRE-PROCESSING FOR CAMERA AND MAP ####################
    # Get tracking info from camera for safeguard check
    frame, markers = threaded_cam.get_marker_positions()
    if frame is None or frame.size == 0 :
        time.sleep(0.01)  # Small pause to let the network buffer catch up
        continue          # Skip the rest of the loop and try getting a fresh frame

    ### little timeout to ensure the camera stream is properly initialized before starting the main loop
    if not startup_done:
        start_time = time.time()
        while time.time() - start_time < RESYNC_DURATION:   #here the waiting time is of 7s, has to be checked case-based
            frame = None
            markers = None
            for _ in range(5):
                f, m = threaded_cam.get_marker_positions()
                if f is not None and hasattr(f, "size") and f.size > 0:
                    frame, markers = f, m
            if frame is None:
                print("Temporary frame drop (ignored)")
                continue
            r.set_speed(0, 0)
        startup_done = True
        continue

    # get tracking infos
    if frame_count % 10 == 0:
        tx, ty, yaw = update_pose()
    else:
        tx, ty, yaw = last_tx, last_ty, last_yaw
    if tx is None:
        continue

    ## memorize path taken by robot
    if frame_count % 2 ==0 and tx is not None:
        path.append((tx, ty))
        for x, y in path:
            ix, iy = world_to_grid(x, y)
            set_cell_if_empty(grid, ix, iy, 1)


    ############## robot explores and recalibrates the position ############
    if state == BASE:

        if time.time() - last_resync > RECALIBRATION_TIMEOUT:
            state = RE_SYNC
            r.set_speed(0, 0)
            last_resync = time.time()
            
            # If we were wall following, note down when the interruption started 
            if mode == WALL_FOLLOWER and wall_follow_start_time is not None:
                resync_interruption_start = time.time()
            
            continue
        
        # -------------------------------------------------------------
        # --- PRIORITY: VIRTUAL GEOFENCE (Boundary Management) --------
        # -------------------------------------------------------------
        in_danger_zone = False

        if tx is not None and ty is not None and yaw is not None:
            # Check boundaries
            out_left = tx < (x_min)
            out_right = tx > (x_max)
            out_bottom = ty < (y_min)
            out_top = ty > (y_max)

            if out_left or out_right or out_bottom or out_top:
                in_danger_zone = True
                
                # Calculate center of the map
                cx, cy = (x_min + x_max) / 2, (y_min + y_max) / 2
                
                # Angle to face the center
                target_angle = math.atan2(cy - ty, cx - tx)
                
                # Calculate steering error
                angle_error = target_angle - yaw
                
                # Normalize angle to [-pi, pi]
                angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
                
                # Sharp proportional turn
                Kp = 1.0 
                ds = Kp * angle_error
                
                left_speed = NORM_SPEED - ds
                right_speed = NORM_SPEED + ds
                
                # Cap speeds
                left_speed = max(min(left_speed, 5), -5)
                right_speed = max(min(right_speed, 5), -5)

        if in_danger_zone:
            r.set_speed(left_speed, right_speed)
            continue

        # -------------------------------------------------------------
        # --- MODE: EXPLORER MODE (Wandering Open Space) --------------
        # -------------------------------------------------------------
        if mode == EXPLORER:
            prox_values = r.get_calibrate_prox()
            
            # If front sensors detect a block nearby, transition to wall following!
            # Using front sensors (ps[0] and ps[7]) exceeding a activation threshold
            if prox_values[0] > 100 or prox_values[7] > 100:
                print("Obstacle detected! Initiating PID Wall Follower...")
                mode = WALL_FOLLOWER
                wall_follow_start_time = time.time()
                # Clear PID memory
                wall_pid.error = 0
                wall_pid.integ = 0
                continue

            # Braitenberg exploration weighted average sensor calculations
            prox_right = (ea * prox_values[0] + eb * prox_values[1] + ec * prox_values[2] + ed * prox_values[3]) / (ea + eb + ec + ed)
            prox_left = (ea * prox_values[7] + eb * prox_values[6] + ec * prox_values[5] + ed * prox_values[4]) / (ea + eb + ec + ed)
            
            ds_left = (NORM_SPEED * prox_right) / PROX_TH   
            ds_right = (NORM_SPEED * prox_left) / PROX_TH  
             
            # Calculate individual motor speeds: s = s0 + ds
            left_speed = NORM_SPEED - ds_left
            right_speed = NORM_SPEED - ds_right
            
            r.set_speed(left_speed, right_speed)
            
        # -------------------------------------------------------------
        # --- MODE: WALL_FOLLOWER MODE (Circumnavigating Obstacles) ---
        # -------------------------------------------------------------
        elif mode == WALL_FOLLOWER:
            # Safety timeout: Check if we have spent enough time mapping this block
            if time.time() - wall_follow_start_time > WALL_FOLLOW_DURATION:
                print("Circumnavigation complete. Resuming Explorer mode...")
                mode = EXPLORER
                continue

            prox_values = r.get_calibrate_prox()
            
            # Read right sensors for PID target (tracking the wall on the robot's right)
            proxR = (wa * prox_values[0] + wb * prox_values[1] + wc * prox_values[2] + wd * prox_values[3]) / (wa + wb + wc + wd)
            
            # Compute PID response
            ds = wall_pid.compute(proxR, PID_WALL_TARGET)
            ds += 0.05 # default bias to curve slightly toward the wall
            
            right_speed = NORM_SPEED + ds
            left_speed = NORM_SPEED - ds
            
            # Clamping behavior for sharp cornering
            if abs(ds) > PID_MAX_DS:
                right_speed = +ds
                left_speed = -ds
                
            # # Extra safeguard: if it loses the wall completely (all sensors near 0), break back to explorer
            # if max(prox_values) < 30:
            #     print("Lost wall contact. Returning to Explorer...")
            #     mode = EXPLORER
            #     continue

            r.set_speed(left_speed, right_speed)
            

    ########### RESYNC ############
    elif state == RE_SYNC:
        r.set_speed(0, 0)
        if resync_start_time is None:
            resync_start_time = time.time()

        while time.time() - resync_start_time < RESYNC_DURATION:
            frame, markers = threaded_cam.get_marker_positions()

            if frame is None:
                print("Failed to get frame during resync.")
                time.sleep(0.05)
                continue

            tx, ty, yaw = update_pose()
            time.sleep(0.02)

        tx, ty, yaw = update_pose()
        print("Re-sync complete:", tx, ty, yaw)
        
        # Adjust the wall following clock if it was interrupted
        if mode == WALL_FOLLOWER and wall_follow_start_time is not None and resync_interruption_start is not None:
            interruption_duration = time.time() - resync_interruption_start
            # Shift the start time forward, effectively "freezing" the countdown during resync
            wall_follow_start_time += interruption_duration
            resync_interruption_start = None # Reset tracker
            print(f"Wall follow timer paused for {interruption_duration:.2f}s due to Re-sync.")

        resync_start_time = None
        state = BASE
        continue

    ########### FRAME INCREMENTATION FOR MAPPING ############
    frame_count +=1
    
    if frame_count % 50 == 0:  # SHOWS updated map every 50 frames
        plt.clf()
        plt.imshow(grid, origin='lower', cmap='gray')
        plt.title("Map")
        plt.pause(0.001)

    if frame_count % 200== 0:     #saves the map each 200 frames, can be later checked and saved with the code "check_map.py"
        np.save("map.npy", grid)
        
    cv2.imshow('RTSP Camera Stream', frame)  #shows camera stream
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
r.clean_up()