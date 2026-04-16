# environment explorer implementation
from itertools import count
from mimetypes import init
from statistics import mode

from networkx import shortest_path
from unifr_api_epuck import wrapper
import sys
import cv2
from vision import ArUcoCamera
import math

import matplotlib.pyplot as plt


import numpy as np
"""
code for checking camera values
"""


##################### initialize tracking (camera streaming) ########################
rtsp_url = f"rtsp://192.168.2.150:8554/cam1"
print(f"Connecting to {rtsp_url}...")
try:
    camera = ArUcoCamera(rtsp_url, marker_size_mm=40)
except Exception as e:
    print(f"Error initializing tracking stream: {e}")
    sys.exit(1)

###################### Robot setup #################################################
#IP
MY_IP = '192.168.2.208' 
''
r = wrapper.get_robot(MY_IP)




###################### CONSTANTS & VARIABLES for behavior ####################################################
MARKER_ID = 0 




###################################### CAMERA #############################################
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


############################## Behavior ##################################################
while r.go_on():

  ################# camera e posizione infos #############################################
    # get tracking infos during all of the behavior
    frame, markers = camera.get_marker_positions()
    if frame is None:
        print("Failed to get frame.")
        break
    if markers is None:
        print("no marker :(")

    rx = None
    ry = None
    tx = None
    ty = None

    # Log marker data 
    if markers :
        if MARKER_ID in markers.keys():
            tx = markers[MARKER_ID]['tvec'][0]
            ty = markers[MARKER_ID]['tvec'][1]
            tz = markers[MARKER_ID]['tvec'][2]
            rx = markers[MARKER_ID]['rvec'][0]
            ry = markers[MARKER_ID]['rvec'][1]
            rz = markers[MARKER_ID]['rvec'][2]
        

            tvec = np.array(markers[MARKER_ID]['tvec'])
            t_cm = tvec.reshape(3,1)

            # Marker position in world frame
            t_wm = R_wc @ t_cm + C_w
            tx = t_wm[0]
            ty = t_wm[1]
                        
            # Convert rvec to rotation matrix
            R_cm, _ = cv2.Rodrigues(markers[0]['rvec'])

            # Marker rotation in world frame
            R_wm = R_wc @ R_cm
            yaw = math.atan2(R_wm[1,0], R_wm[0,0])
            yaw_deg = np.degrees(yaw)
            
            # compute angle of current orientation
            head = yaw

            print(f"Marker {MARKER_ID} position in world: ({tx}, {ty}), yaw: {yaw_deg} degrees", "yaw in radians: ", yaw)
        else:
            print(f"Marker {MARKER_ID} not detected.")


    




    # Show camera frame
    cv2.imshow('RTSP Camera Stream', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break





########################### Clean up ##############################
#np.save("map.npy", grid)  # save map grid to file
camera.release()
cv2.destroyAllWindows()

r.clean_up()
