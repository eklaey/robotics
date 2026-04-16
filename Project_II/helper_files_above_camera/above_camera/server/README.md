# Camera tracking

## Server setup

1. Go to the server folder

2. Copy install_server.sh and mediamtx.yml to raspi

u: pi
p: 1203Video&

    sudo apt update
    sudo apt upgrade -y
    chmod +x install_server.sh
    ./install_server.sh

3. You should have 2 cameras streams. 

    rtsp://192.168.2.150:8554/cam1

    rtsp://192.168.2.150:8554/cam2

4. Test the streaming using the python code client.py (if you haven't done the calibration step below the code issues a warning)

If things do not work as expected, modify the mediamtx.yml config file (scroll to the bottom)

    nano mediamtx.yml
    sudo cp mediamtx.yml /usr/local/etc/mediamtx.yml
    sudo systemctl daemon-reload
    sudo systemctl restart mediamtx



## Camera calibration

1. On your local computer, run the calibration.py python script

2. Capture 20 frames with changing orientations of the aruco board (press 'c' to capture)

3. test the aruco localisation using the python code client.py (there should be no warning any more and an arudo marker should show data overlay)

## Tracking test

1. Put an aruco marker on the robot

2. update rtsp_url, ID (marker ID) and MY_IP variables in the tracking code

3. run the code. The robot should circle around the camera origin on a 20cm radius.



