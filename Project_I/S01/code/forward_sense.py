from unifr_api_epuck import wrapper
import numpy as np

MY_IP = '192.168.2.205' # modify the last number with the last 3 digits of the robot ID (on sticker)
robot = wrapper.get_robot(MY_IP)

NORM_SPEED = 1.5
PROX_TH = 250
#PROX_TH /= 4

robot.init_sensors()
robot.calibrate_prox()

prox_array = []

#---------------------------------------------
def equals(x, y):
    threshold = 1
    if x - y < threshold:
        return True
    else:
        return False   
#--------------------------------------------

while robot.go_on():
    
    #------------------------------#
    prox_values = robot.get_calibrate_prox()
    
    prox = (prox_values[0] + prox_values[7])/2
    ds = (NORM_SPEED * prox) / PROX_TH
    speed = NORM_SPEED - ds
    robot.set_speed(speed)
    #------------------------------#
    
    prev_prox_avg = np.mean(prox_array[-10:]) if len(prox_array) >= 10 else prox
        
    if prev_prox_avg > PROX_TH and equals(prox, prev_prox_avg):
        robot.set_speed(0)
        print("---Reached equilibrium at prox:", prox)
        break
    else:
        print("Current prox:", prox)
        prox_array.append(prox)
        prox_array.pop(0)
        
robot.clean_up()
