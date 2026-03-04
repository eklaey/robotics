from unifr_api_epuck import wrapper
import time

MY_IP = '192.168.2.205' # modify the last number with the last 3 digits of the robot ID (on sticker)
robot = wrapper.get_robot(MY_IP)

NORM_SPEED = 1.5
PROX_TH = 250
PROX_TH /= 4

a, b, c, d = 1, 1.5, 2, 4

robot.init_sensors()
robot.calibrate_prox()

current_prox = robot.get_calibrate_prox()[0]

#---------------------------------------------
def equals(x, y):
    threshold = 100
    if x - y < threshold:
        return True
    else:
        return False   
#--------------------------------------------

while robot.go_on():
    prox_values = robot.get_calibrate_prox()
    
    prox_right = (a * prox_values[0] + b * prox_values[1] + c * prox_values[2] + d * prox_values[3]) / (a + b + c + d)
    prox_left = (a * prox_values[7] + b * prox_values[6] + c * prox_values[5] + d * prox_values[4]) / (a + b + c + d)
    
    avg_prox = (prox_right + prox_left) / 2
    
    ds = (NORM_SPEED * avg_prox) / PROX_TH
    speed = NORM_SPEED - ds
    robot.set_speed(speed)
    
    prev_avg_prox = current_prox
    current_prox = robot.get_calibrate_prox()[0]
    
    if avg_prox < PROX_TH and equals(current_prox, prev_avg_prox):
        robot.set_speed(0)
        print("Reached equilibrium at prox:", current_prox)
        
    
    

robot.clean_up()
