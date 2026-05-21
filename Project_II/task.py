from unifr_api_epuck import wrapper

from vision import ArUcoCamera
from threaded_camera import ThreadedCamera
from pid import PID

import os
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import time
import sys
from collections import deque
import networkx as nx

############# initialize tracking (camera streaming) ###################
rtsp_url = f"rtsp://192.168.2.150:8554/cam1"
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

###################### ROBOT SETUP #####################################
########################################################################
MY_IP = '192.168.2.202' # Robot IP to be change accordingly to the used one
r = wrapper.get_robot(MY_IP)
MARKER_ID = 19    # ArUco marker ID to be change accordingly to the used one

####################### variables ######################################
NORM_SPEED = 1.5

x_min = x_max = y_min = y_max = 0
resolution = 0.02 
path = []   # stores (x, y) robot positions 

startup_done = False

last_resync = 0
has_not_resync = True
resync_start_time = None
RESYNC_DURATION = 7.0
RECALIBRATION_TIMEOUT = 29.0

# Geofencing
BUFFER = 0.1
Kp_fence = 0.5

# Braitenberg EXPLORER
PROX_TH = 250
ea, eb, ec, ed = 1, 2, 2, -1

# PID Parameters for Wall Following
PID_MAX_DS = 1.5
PID_WALL_TARGET = 200
wa, wb, wc, wd = 4, 2, 1, 0

DEFAULT_DS_OFFSET = 0.05 #originally 0.05, 207 0.1
K = 0.0055              #originally 0.0055, 207 0.0085
T_D = 0.2         #originally 0.2, 207 0.3
T_I = 9999999999

wall_pid = PID(K, T_I, T_D)

# Behavior tracking variables
resync_interruption_start = None

# Wall Following timeouts and thresholds
WALL_DETECTION_THRESHOLD = 150
wall_following_side = "RIGHT"
wall_follow_start_time = 0
WALL_FOLLOW_DURATION = 20.0
lost_wall_start_time = 0
LOST_WALL_TIMEOUT = 1.0 
wall_exit_time = 0
WALL_EXIT_TIMEOUT = 3.0

# Object LOVER parameters
TARGET_DISTANCE = 60
ANGULAR_GAIN = 1.2
DISTANCE_GAIN = 1.0

USE_COLOR_DETECTION = True
COLOR_DETECTION_THRESHOLD = 750 # Minimum area of color blob to be considered goal

DETECTION_CONFIDENCE = 0.9

# Navigation and Mapping
num_goals_found = 0
red_goal_grid = None
green_goal_grid = None
planned_path_world = [] # Will hold the physical (wx, wy) coordinates to drive to

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

def get_smoothed_prox():
    """Fetches raw prox values, applies a moving average, and returns smoothed values."""
    raw_values = r.get_calibrate_prox()
    smoothed_values = []
    
    for i in range(8):
        # Push the new raw reading into the queue for sensor 'i'
        prox_history[i].append(raw_values[i])
        
        # Calculate the average of the window
        avg_val = sum(prox_history[i]) / len(prox_history[i])
        smoothed_values.append(avg_val)
        
    return smoothed_values
def image_letterboxed():
    """ Letterboxes image coming from robot's forward facing camera """
    raw_img = np.array(r.get_camera()) # Shape: (3, 120, 160)
    _, height, _ = raw_img.shape # (C, H, W) format from robot camera
    
    # Define the vertical crop limits (e.g., cut off top 30% and bottom 20%)
    y_start = int(120 * 0.30) 
    y_end = int(120 * 0.80)  
                
    masked_img = raw_img.copy()
    masked_img[:, 0:y_start, :] = 0
    masked_img[:, y_end:height, :] = 0
    return masked_img
def map_block_in_front(tx, ty, yaw, block_type):
    """ Projects block onto map from the robot's center and paints a 3x3 patch on the grid """
    block_x = tx + 0.05 * math.cos(yaw)
    block_y = ty + 0.05 * math.sin(yaw)
    bx, by = world_to_grid(block_x, block_y)
    
    # Paint a 3x3 grid patch so it is highly visible on the map
    for i in range(-1, 2):
        for j in range(-1, 2):
            if 0 <= bx+i < grid.shape[1] and 0 <= by+j < grid.shape[0]:
                grid[by+j, bx+i] = block_type
def calculate_return_path(start_grid, target_grid, current_grid):
    print("Converting Map to Graph...")
    # Create a graph where every grid cell is connected to its Up/Down/Left/Right neighbors
    G = nx.grid_2d_graph(current_grid.shape[0], current_grid.shape[1])
    
    # Remove all obstacle cells (4) so the path cannot go through them
    for y in range(current_grid.shape[0]):
        for x in range(current_grid.shape[1]):
            if current_grid[y, x] == 4:
                if (y, x) in G:
                    G.remove_node((y, x))
    try:
        # Find the shortest path (Dijkstra algorithm)
        # NetworkX grid nodes are represented as (y, x)
        path = nx.shortest_path(G, source=(start_grid[1], start_grid[0]), target=(target_grid[1], target_grid[0]))
        
        # Convert the (y, x) grid path back into physical (wx, wy) coordinates
        world_path = []
        for p in path:
            wx, wy = grid_to_world(p[1], p[0]) # p is (y, x)
            world_path.append((wx, wy))
            
        print(f"Path successfully found! {len(world_path)} waypoints.")
        return world_path
    except nx.NetworkXNoPath:
        print("CRITICAL: No valid path found through the maze!")
        return []
                
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
y_min = ty - 0.40
y_max = ty + 0.40

# create grid map
nx = int((x_max - x_min) / resolution)
ny = int((y_max - y_min) / resolution)
grid = np.zeros((ny, nx))
print(f"Map initialized successfully: x[{x_min:.2f}, {x_max:.2f}] y[{y_min:.2f}, {y_max:.2f}]")

###################### SENSOR & CAMERA CALIBRATION ###################################
r.init_sensors()
r.calibrate_prox()

WINDOW_SIZE = 4    # Moving Average Filter Buffers
prox_history = [deque(maxlen=WINDOW_SIZE) for _ in range(8)]

r.initiate_model()
os.makedirs("./img", exist_ok=True)
r.init_camera("./img")

# Custom Map Colors: 0=Empty(White), 1=Path(Gray), 2=RedGoal(Red), 3=GreenGoal(Green), 4=Obstacle(Black)
cmap_custom = mcolors.ListedColormap(['white', 'lightgray', 'red', 'green', 'black'])
norm_custom = mcolors.BoundaryNorm([-.5, 0.5, 1.5, 2.5, 3.5, 4.5], cmap_custom.N)

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
            out_left = tx < (x_min + BUFFER)
            out_right = tx > (x_max - BUFFER)
            out_bottom = ty < (y_min + BUFFER)
            out_top = ty > (y_max - BUFFER)

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
                
                ds = Kp_fence * angle_error
                
                left_speed = NORM_SPEED - ds
                right_speed = NORM_SPEED + ds

        if in_danger_zone:
            r.set_speed(left_speed, right_speed)
            continue

        # -------------------------------------------------------------
        # --- MODE: EXPLORER MODE (Wandering & Scanning) --------------
        # -------------------------------------------------------------
        if mode == EXPLORER:
            img = image_letterboxed()
            
            if USE_COLOR_DETECTION:
                detections = r.get_colordetection(img)
                red_label, green_label = "Red", "Green"
            else:
                # detections = r.get_detection(img, conf_thresh=DETECTION_CONFIDENCE)
                red_label, green_label = "Red Block", "Green Block"   
                         
            # Filter for Red and Green
            target = None
            target_color = None
            if detections:
                reds = [d for d in detections if d.label == "Red"]
                greens = [d for d in detections if d.label == "Green"]
                
                if red_goal_grid is not None:
                    reds = []
                if green_goal_grid is not None:
                    greens = []
                
                # Pick the largest area detected
                if reds and greens:
                    best_red = max(reds, key=lambda d: d.area)
                    best_green = max(greens, key=lambda d: d.area)
                    if best_red.area > best_green.area:
                        target, target_color = best_red, 2 # 2 is Red in our map
                    else:
                        target, target_color = best_green, 3 # 3 is Green in our map
                elif reds:
                    target, target_color = max(reds, key=lambda d: d.area), 2
                elif greens:
                    target, target_color = max(greens, key=lambda d: d.area), 3

            # If a significant color blob is seen, switch to LOVER
            valid_target_found = False
            if target is not None:
                if USE_COLOR_DETECTION:
                    # Color blobs must pass the area threshold
                    if target.area > COLOR_DETECTION_THRESHOLD:
                        valid_target_found = True
                else:
                    # Object detection already passed the confidence threshold in the API call
                    valid_target_found = True
            
            if valid_target_found:
                print(f"Goal detected! Switching to LOVER mode ({target.label}).")
                mode = LOVER
                continue

            # Normal Braitenberg Explorer in case no color detected            
            prox_values = get_smoothed_prox()
            
            if (not((time.time() - wall_exit_time) < WALL_EXIT_TIMEOUT) and 
                (   np.mean(prox_values[0] + prox_values[1]) > WALL_DETECTION_THRESHOLD or 
                    np.mean(prox_values[7] + prox_values[6]) > WALL_DETECTION_THRESHOLD)):
                print("Obstacle detected! Initiating PID Wall Follower...")
                mode = WALL_FOLLOWER
                wall_follow_start_time = time.time()
                # Clear PID memory
                wall_pid.error = 0
                wall_pid.integ = 0
                
                # Compare front-left sensors vs front-right sensors
                if (np.mean(prox_values[7] + prox_values[6])) > (np.mean(prox_values[0] + prox_values[1])):
                    wall_following_side = "LEFT"
                else:
                    wall_following_side = "RIGHT"
                    
                print(f"Following wall on the {wall_following_side} side.")
                
                r.set_speed(0, 0)
                continue

            # Braitenberg exploration weighted average sensor calculations
            prox_right = (ea * prox_values[0] + eb * prox_values[1] + ec * prox_values[2] + ed * prox_values[3]) / (ea + eb + ec + ed)
            prox_left = (ea * prox_values[7] + eb * prox_values[6] + ec * prox_values[5] + ed * prox_values[4]) / (ea + eb + ec + ed)
            
            ds_left = (NORM_SPEED * prox_right) / PROX_TH   
            ds_right = (NORM_SPEED * prox_left) / PROX_TH  
             
            # Calculate individual motor speeds: s = s0 + ds
            left_speed = NORM_SPEED - ds_left
            right_speed = NORM_SPEED - ds_right
            
            #Clamp speeds to max
            left_speed = max(min(left_speed, 2 * NORM_SPEED), -2 * NORM_SPEED)
            right_speed = max(min(right_speed, 2 * NORM_SPEED), -2 * NORM_SPEED)
            
            r.set_speed(left_speed, right_speed)
            
        # -------------------------------------------------------------
        # --- MODE: LOVER MODE (Object Lover based on Color) ----------
        # -------------------------------------------------------------
        elif mode == LOVER:
            img = image_letterboxed()
            if USE_COLOR_DETECTION:
                detections = r.get_colordetection(img)
                label_map = {"Red": 2, "Green": 3}
            else:
                # detections = r.get_detection(img, conf_thresh=DETECTION_CONFIDENCE)
                label_map = {"Red Block": 2, "Green Block": 3}    
                        
            # Keep tracking the color we locked onto
            target_str = "Red" if target_color == 2 else "Green"
            search_str = target_str if USE_COLOR_DETECTION else f"{target_str} Block"
            
            valid_targets = [d for d in detections if d.label == search_str]
            
            if not valid_targets:
                print("Lost visual on goal. Returning to EXPLORER.")
                mode = EXPLORER
                continue
                
            target = max(valid_targets, key=lambda d: d.area)
            
            # Check if we have arrived at the block
            tof_distance = r.get_tof()
            prox_values = get_smoothed_prox()
            
            if tof_distance < TARGET_DISTANCE or (prox_values[0] > 500 or prox_values[7] > 500):
                print(f"Goal Reached! Mapping {target_str} block and circumnavigating.")
                # Get the grid coordinate of the block to map it
                block_x = tx + 0.08 * math.cos(yaw)
                block_y = ty + 0.08 * math.sin(yaw)
                bx, by = world_to_grid(block_x, block_y)
                
                # Map and save location of the block on the grid
                map_block_in_front(tx, ty, yaw, target_color)
                
                # Save specific goal coordinates for navigation
                if target_color == 2 and red_goal_grid is None:
                    red_goal_grid = (bx, by)
                    num_goals_found += 1
                elif target_color == 3 and green_goal_grid is None:
                    green_goal_grid = (bx, by)
                    num_goals_found += 1
                
                # --- PHASE 2 TRIGGER CHECK ---
                if red_goal_grid is not None and green_goal_grid is not None:
                    print("+++ BOTH GOALS FOUND! INITIATING PHASE 2 NAVIGATION +++")
                    r.set_speed(0, 0)
                    
                    # We are currently at the 2nd goal. We need to go back to the 1st goal.
                    # Figure out which one we are at right now
                    current_grid_pos = world_to_grid(tx, ty)
                    if target_color == 2: # We are at Red, go to Green
                        target_destination = green_goal_grid
                    else:                 # We are at Green, go to Red
                        target_destination = red_goal_grid
                        
                    planned_path_world = calculate_return_path(current_grid_pos, target_destination, grid)
                    #mode = PATH_FINDER
                    continue
                
                # If we haven't found both yet, keep exploring (Wall Follow first)
                mode = WALL_FOLLOWER
                wall_follow_start_time = time.time()
                wall_pid.error = 0; wall_pid.integ = 0
                continue


            # OBJECT LOVER (Continuous, No Dead-band)
            camera_center = img.shape[1] / 2
            
            distance_ds = ((tof_distance - TARGET_DISTANCE) / TARGET_DISTANCE) * DISTANCE_GAIN
            angular_ds = ((target.x_center - camera_center) / camera_center) * ANGULAR_GAIN
            
            left_speed = distance_ds + angular_ds
            right_speed = distance_ds - angular_ds
            
            # Clamp speeds to max
            left_speed = max(min(left_speed, 2 * NORM_SPEED), -2 * NORM_SPEED)
            right_speed = max(min(right_speed, 2 * NORM_SPEED), -2 * NORM_SPEED)
            
            r.set_speed(left_speed, right_speed)    
            
        # -------------------------------------------------------------
        # --- MODE: WALL FOLLOWER MODE (Circumnavigating Obstacles) ---
        # -------------------------------------------------------------
        elif mode == WALL_FOLLOWER:
            # Safety timeout: Check if we have spent enough time mapping this block
            if time.time() - wall_follow_start_time > WALL_FOLLOW_DURATION:
                print("Circumnavigation complete. Resuming Explorer mode...")
                mode = EXPLORER
                wall_exit_time = time.time()
                continue
            
            #prox_values = get_smoothed_prox()
            prox_values = r.get_calibrate_prox() # For wall loss detection, we want the raw values to be more sensitive
            
            # Extra safeguard: if it loses the wall completely (all sensors near 0), break back to explorer
            if max(prox_values) < 20:
                # print("Lost wall contact. Returning to Explorer...")
                # mode = EXPLORER
                # continue
                if lost_wall_start_time is None:
                    lost_wall_start_time = time.time()
                
                elif time.time() - lost_wall_start_time > LOST_WALL_TIMEOUT:
                    print("Lost wall confirmed. Returning to Explorer...")
                    mode = EXPLORER
                    lost_wall_start_time = 0
                    continue
            else:
                # Wall detected --> Reset the timer
                lost_wall_start_time = 0
            
            # --- NEW: Bidirectional PID Computation ---
            if wall_following_side == "RIGHT":
                # Standard right-wall tracking
                prox_side = (wa * prox_values[0] + wb * prox_values[1] + wc * prox_values[2] + wd * prox_values[3]) / (wa + wb + wc + wd)
                ds = wall_pid.compute(prox_side, PID_WALL_TARGET)
                ds += DEFAULT_DS_OFFSET  # Small forward bias to keep it moving 
                
                right_speed = NORM_SPEED + ds
                left_speed = NORM_SPEED - ds
                
                # Clamping for sharp cornering (not necessary perhaps)
                if abs(ds) > PID_MAX_DS:
                    right_speed = +ds
                    left_speed = -ds
                    
            elif wall_following_side == "LEFT":
                # Mirror the sensors for the Left wall (0->7, 1->6, 2->5, 3->4)
                prox_side = (wa * prox_values[7] + wb * prox_values[6] + wc * prox_values[5] + wd * prox_values[4]) / (wa + wb + wc + wd)
                ds = wall_pid.compute(prox_side, PID_WALL_TARGET)
                ds += DEFAULT_DS_OFFSET  # Small forward bias to keep it moving
                
                # Swap the speeds for the Left side
                left_speed = NORM_SPEED + ds
                right_speed = NORM_SPEED - ds
                
                # Clamping for sharp cornering (not necessary perhaps)
                if abs(ds) > PID_MAX_DS:
                    left_speed = +ds
                    right_speed = -ds

            # Clamp speeds to max
            left_speed = max(min(left_speed, 2 * NORM_SPEED), -2 * NORM_SPEED)
            right_speed = max(min(right_speed, 2 * NORM_SPEED), -2 * NORM_SPEED)

            r.set_speed(left_speed, right_speed)
        
        # -------------------------------------------------------------
        # --- MODE: PATH FINDER MODE (Navigation) ---------------------
        # -------------------------------------------------------------
        elif mode == PATH_FINDER:
            if not planned_path_world:
                print("DESTINATION REACHED! Phase 2 Complete.")
                r.set_speed(0, 0)
                # You can optionally break the while loop here to end the script
                continue
                
            # Get the current waypoint
            target_wx, target_wy = planned_path_world[0]
            
            # Check how far we are from the waypoint
            dist_to_waypoint = math.sqrt((target_wx - tx)**2 + (target_wy - ty)**2)
            
            # If we are closer than 4cm, pop the waypoint and target the next one
            if dist_to_waypoint < 0.04:
                planned_path_world.pop(0)
                continue
                
            # Calculate angle to the waypoint
            target_angle = math.atan2(target_wy - ty, target_wx - tx)
            angle_error = target_angle - yaw
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            
            # Steer towards the waypoint (Proportional steering)
            Kp_nav = 1.5
            ds = Kp_nav * angle_error
            
            # Drive forward while turning
            # We slow down the base speed so it doesn't overshoot waypoints
            nav_speed = NORM_SPEED * 0.8 
            left_speed = max(min(nav_speed - ds, 4.0), -4.0)
            right_speed = max(min(nav_speed + ds, 4.0), -4.0)
            
            r.set_speed(left_speed, right_speed)
            

    ########### RESYNC ############
    elif state == RE_SYNC:
        print("Re-syncing pose with ArUco marker...")
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
        
        # Adjust the wall clocks if interrupted by re-sync
        if (mode == WALL_FOLLOWER and 
            (wall_follow_start_time != 0 or 
             lost_wall_start_time != 0 or
             wall_exit_time != 0) and 
            resync_interruption_start is not None):
            
            interruption_duration = time.time() - resync_interruption_start
            
            # Shift the time counters forward, effectively "freezing" the countdown during resync
            if wall_follow_start_time != 0:
                wall_follow_start_time += interruption_duration
            elif lost_wall_start_time != 0:
                lost_wall_start_time += interruption_duration
            elif wall_exit_time != 0:
                wall_exit_time += interruption_duration
                
            resync_interruption_start = None # Reset tracker
            print(f"Wall follow timer paused for {interruption_duration:.2f}s due to Re-sync.")

        resync_start_time = None
        state = BASE
        continue

    ########### FRAME INCREMENTATION FOR MAPPING ############
    frame_count +=1
    
    if frame_count % 50 == 0:  # SHOWS updated map every 50 frames
        plt.clf()
        #plt.imshow(np.fliplr(grid), origin='upper', cmap='gray')
        plt.imshow(np.fliplr(grid), origin='upper', cmap=cmap_custom, norm=norm_custom)
        plt.title("Map")
        plt.pause(0.001)
        
    cv2.imshow('RTSP Camera Stream', frame)  #shows camera stream
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

np.save("map.npy", grid) # can be viewd with check_map.py
camera.release()
cv2.destroyAllWindows()
r.clean_up()