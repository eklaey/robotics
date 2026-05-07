### imports

from unifr_api_epuck import wrapper
import cv2
from vision import ArUcoCamera
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
    '''
    robot behavior
    '''
       
    cv2.imshow('RTSP Camera Stream', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


camera.release()
cv2.destroyAllWindows()
r.clean_up()