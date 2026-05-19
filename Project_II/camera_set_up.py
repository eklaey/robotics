### imports

from unifr_api_epuck import wrapper
import cv2
from vision import ArUcoCamera
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import sys

############# initialize tracking (camera streaming) ###################
rtsp_url = f"rtsp://192.168.2.150:8554/cam1"
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

frame = None
markers = None


# last pose
last_tx, last_ty, last_yaw = None, None, None

frame_count = 0

###################### Robot setup #####################################
MY_IP = '192.168.2.208' 
''
r = wrapper.get_robot(MY_IP)

################# variables ################
MARKER_ID = 0    # ArUco marker ID to be change accordingly to the used one

NORM_SPEED = 1.8

x_min = x_max = y_min = y_max = 0
resolution = 0.02 
path = []   # stores (x, y) robot positions 


startup_done = False

last_resync = 0
has_not_resync = True
resync_start_time = None
RESYNC_DURATION = 7.0

###### simple state machine having basic and resync behaviors ##########
BASE = 1
RE_SYNC = 2

state = BASE

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
        new_frame, new_markers = camera.get_marker_positions()
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
###################### MAP PREPARATION #########################################
tx, ty, yaw = update_pose()
if tx is not None and ty is not None and yaw is not None :  #define borders of the map
    x_min = tx - 0.45
    x_max = tx + 0.45
    y_min = ty - 0.30
    y_max = ty + 0.30
# create grid map
nx = int((x_max - x_min) / resolution)
ny = int((y_max - y_min) / resolution)
grid = np.zeros((ny, nx))
print("map made", "x_max", x_max, "x_min", x_min)

######################## GO_ON LOOP ####################################################
while r.go_on():

    ### little timeout to ensure the camera stream is properly initialized before starting the main loop
    if not startup_done:
        start_time = time.time()
        while time.time() - start_time < RESYNC_DURATION:   #here the waiting time is of 7s, has to be checked case-based
            frame = None
            markers = None
            for _ in range(5):
                f, m = camera.get_marker_positions()
                if f is not None and hasattr(f, "size") and f.size > 0:
                    frame, markers = f, m
            if frame is None:
                print("Temporary frame drop (ignored)")
                continue
            r.set_speed(0, 0)
        startup_done = True
        continue

    # get tracking infos
    if frame_count % 1 == 0:
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


    ############## robot goes straight and recalibrates the position each 20 seconds ##########################
    if state == BASE:

        if time.time() - last_resync > 19:
            state = RE_SYNC
            r.set_speed(0, 0)
            last_resync = time.time()


        if state == RE_SYNC:
            continue

        r.set_speed(NORM_SPEED,NORM_SPEED)
            

    ########### RESYNC ############
    elif state == RE_SYNC:
        r.set_speed(0, 0)
        if resync_start_time is None:
            resync_start_time = time.time()

        while time.time() - resync_start_time < RESYNC_DURATION:
            frame, markers = camera.get_marker_positions()

            if frame is None:
                print("Failed to get frame during resync.")
                time.sleep(0.05)
                continue

            tx, ty, yaw = update_pose()
            time.sleep(0.02)

        tx, ty, yaw = update_pose()

        print("Re-sync complete:", tx, ty, yaw)
        resync_start_time = None
        state = BASE
        continue


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