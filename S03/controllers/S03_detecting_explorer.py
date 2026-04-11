from unifr_api_epuck import wrapper
import sys, random, signal
import numpy as np

# on terminal : 
# conda activate robotics
# python3 -m unifr_api_epuck -g
# python S03_detecting_eplorer.py 192.168.2.207
# or to run multiple controllers : 
# ./run_multiple.sh 206 205

if __name__ == "__main__":
    """
    if arguments in the command line --> IRL
    no arguemnts --> use Webots
    """
    ip_addr = None
    if len(sys.argv) == 2:
        ip_addr = sys.argv[1]
    
    robot = wrapper.get_robot(ip_addr)
    
    def handler(signum, frame):
        robot.clean_up()

    signal.signal(signal.SIGINT, handler)   
    
    NORM_SPEED = 1.2
    MAX_PROX = 250

    robot.init_sensors()
    robot.calibrate_prox()
    robot.initiate_model()

    robot.init_camera("img")
    
    stepcounter = 0
    detectcounter = 0   
    n = 5

    # initialize client communication (server needs to be started)
    robot.init_client_communication()
    detection_msg = ip_addr
    received_msg = ''

    while robot.go_on():
        
        # start camera
        if stepcounter % n == 0 :
            robot.init_camera()

        # take picture -> detect objects
        if stepcounter % n == 1 :
            img = np.array(robot.get_camera())
            detections = robot.get_detection(img, 0.2)
            
            # object detected -> create new message to send
            if len(detections) > 0:
                detection_msg = ''
                detectcounter = 0   
                for object in detections:
                    detection_msg = detection_msg + " " + object.label
                    print(ip_addr + "DETECTION : " + detection_msg)

            # nothing detected -> previous message is erased after a delay
            elif detectcounter > 2*n:
                detection_msg = ''

            speed_left, speed_right = 0, 0
            robot.disable_camera() # BUG !! will be working soon...
            robot.send_msg(detection_msg)
        
        else :
            # check for new message
            if robot.has_receive_msg():
                received_msg = robot.receive_msg()
                print("RECEIVED : " + received_msg)

            # get IR sensor values
            prox = robot.get_calibrate_prox()
        
            # behaviours: compute speed according to prox values and current state 
            prox_left = (4 * prox[7] + 2 * prox[6] + prox[5]) / 7
            prox_right = (4 * prox[0] + 2 * prox[1] + prox[2]) / 7

            ds_left = (NORM_SPEED * prox_left) / MAX_PROX
            ds_right = (NORM_SPEED * prox_right) / MAX_PROX

            speed_left = NORM_SPEED - ds_right
            speed_right = NORM_SPEED - ds_left        
                        
        # set speed
        robot.set_speed(speed_left, speed_right)
        
        # compare detection to received message -> LEDS
        if stepcounter % n == n-1:
            if detection_msg != '' and received_msg == detection_msg:
                robot.enable_all_led()
            else:
                robot.disable_all_led()
        
        stepcounter += 1
        detectcounter += 1    
        

    robot.clean_up()
    
