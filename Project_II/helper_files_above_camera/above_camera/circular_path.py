# robot following a circular path
from unifr_api_epuck import wrapper
import sys
import cv2
from vision import ArUcoCamera
import math

import numpy as np



# initialize tracking (pip install opencv-contrib-python)
rtsp_url = f"rtsp://192.168.2.150:8554/cam2"
print(f"Connecting to {rtsp_url}...")
try:
    camera = ArUcoCamera(rtsp_url, marker_size_mm=40)
except Exception as e:
    print(f"Error initializing tracking stream: {e}")
    sys.exit(1)

MARKER_ID = 0
MY_IP = '192.168.2.208'
r = wrapper.get_robot(MY_IP)

NORM_SPEED = 2.0
TH_PROX = 250

# states
ROUND = 3


# thresholds to increase counters
THRESH_SPEED = 0.1
THRESH_PROX_LOW = 70

r.init_sensors()

state = ROUND
r.enable_all_led()


# Camera to world rotation (overhead camera)
R_wc = np.array([
    [1,  0,  0],
    [0, -1,  0],
    [0,  0, -1]
])

# Camera position in world (1.1m above plane)
C_w = np.array([[0],
                [0],
                [1.1]])

while r.go_on():

    # get tracking infos
    frame, markers = camera.get_marker_positions()
    if frame is None:
        print("Failed to get frame.")
        break

    rx = None
    ry = None
    tx = None
    ty = None
    # Log marker data 
    if markers:
        print(f"Detected markers: {list(markers.keys())}")
        # Example access: markers[id]['tvec']
        if MARKER_ID in markers.keys():
            tx = markers[MARKER_ID]['tvec'][0]
            ty = markers[MARKER_ID]['tvec'][1]
            tz = markers[MARKER_ID]['tvec'][2]
            rx = markers[MARKER_ID]['rvec'][0]
            ry = markers[MARKER_ID]['rvec'][1]
            rz = markers[MARKER_ID]['rvec'][2]
        
        
    # example tracking-based control     
    if state == ROUND and markers:
        
        # #####################
        # Linear algebra transformations to get marker position and heading
        
        # Convert rvec to rotation matrix
        R_cm, _ = cv2.Rodrigues(markers[0]['rvec'])

        # Marker rotation in world frame
        R_wm = R_wc @ R_cm
        yaw = math.atan2(R_wm[1,0], R_wm[0,0])
        yaw_deg = np.degrees(yaw)

        # Marker translation in camera frame
        tvec = np.array(markers[0]['tvec'])
        t_cm = tvec.reshape(3,1)

        # Marker position in world frame
        t_wm = R_wc @ t_cm + C_w
        tx = t_wm[0]
        ty = t_wm[1]

        # compute angle of current orientation
        head = yaw
        
        # ##################
        # Circling behaviour
        
        # compute target angle perpendicular to position vector angle (will circle around origin)
        target = math.pi + math.atan2(ty,tx)
        if target < -math.pi:
            target += 2*math.pi
        elif target > math.pi :
            target -= 2*math.pi
        target_deg = np.degrees(target)
        
        # compute heading deviation 
        error = head-target
        if error < -math.pi:
            error += 2*math.pi
        elif error > math.pi :
            error -= 2*math.pi
            
        # compute distance to circle at around 20cm around the camera origin 
        distance = math.sqrt(tx*tx+ty*ty)
        
        ds =  2.5*error + 30*(distance-0.2)
        
        speed_left = NORM_SPEED + ds
        speed_right = NORM_SPEED - ds       

        r.disable_all_led()
        r.enable_led(1)
        
        print("head {} target {} error {} target deg {}".format(head,target,error,target_deg))
        print("distance {} ds {}".format(distance,ds))

    #print("rotation {} {} {} translation {} {} {}".format(rx,ry,rz,tx,ty,tz))


    r.set_speed(speed_left, speed_right)
    
    cv2.imshow('RTSP Camera Stream', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()

r.clean_up()

