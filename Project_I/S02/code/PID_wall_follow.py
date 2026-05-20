# wall following controller 

from unifr_api_epuck import wrapper
import os # for log files
from collections import deque
import signal

MY_IP = '192.168.2.202'
robot = wrapper.get_robot(MY_IP)

def handler(signum, frame):
    robot.clean_up()

def get_smoothed_prox():
    """Fetches raw prox values, applies a moving average, and returns smoothed values."""
    raw_values = robot.get_calibrate_prox()
    smoothed_values = []
    
    for i in range(8):
        # Push the new raw reading into the queue for sensor 'i'
        prox_history[i].append(raw_values[i])
        
        # Calculate the average of the window
        avg_val = sum(prox_history[i]) / len(prox_history[i])
        smoothed_values.append(avg_val)
        
    return smoothed_values

signal.signal(signal.SIGINT, handler)

# general parameters
PID_MAX_DS = 1.5
NORM_SPEED = 2
PID_WALL_TARGET = 200
a = 4
b = 2
c = 1
d = 0

# PID parameters
K = 0.0055 
T_D = 0.2
T_I = 9999999999  #optional

class PID:

    TIME_STEP = 64

    def __init__(self, k, t_i, t_d):
        self.error = 0
        self.deriv = 0
        self.integ = 0
        self.K = k
        self.T_I = t_i
        self.T_D = t_d

    def compute(self,prox,target):    
        prev_err = self.error
        self.error = prox - target

        self.deriv = (self.error - prev_err)*1000/self.TIME_STEP
        self.integ += self.error*self.TIME_STEP/1000

        #return self.K * ( self.error + 1.0 / self.T_I * self.integ + self.T_D * self.deriv)
        return self.P() + self.I() + self.D()

    def P(self) :
        return self.K * self.error    

    def I(self) :
        return self.K * (self.integ/self.T_I)    

    def D(self) :
        return self.K * (self.T_D * self.deriv)    
    

# open file for writing (adding a number if already exists)
n = 0
while os.path.exists("../recordings/logPID_{}.csv".format(n)):
    n += 1
data = open("../recordings/logPID_{}.csv".format(n), "w")

if data == None:
    print('Error opening data file!\n')
    quit

#write header in CSV file
data.write('step,K,T_I,T_D,target,error,P,I,D,ds,left speed,right speed\n')

# create pid instance
pid = PID(K, T_I, T_D)

step = 0

robot.init_sensors()
robot.calibrate_prox()

WINDOW_SIZE = 10    # Moving Average Filter Buffers (8 sensors, storing 10 values each)
prox_history = [deque(maxlen=WINDOW_SIZE) for _ in range(8)]

robot.sleep(2)

#infinite loop
while robot.go_on():
    #ps = robot.get_calibrate_prox()
    ps = get_smoothed_prox()
    
    proxR = (a * ps[0] + b * ps[1] + c * ps[2] + d * ps[3]) / (a+b+c+d);
                      
    # compute PID response according to IR sensor value
    ds = pid.compute(proxR,PID_WALL_TARGET);      
          
    # make the robot turn towards the wall by default    
    ds += .05

    speedR = NORM_SPEED + ds
    speedL = NORM_SPEED - ds
    
    # "clamping" function for corners
    if abs(ds) > PID_MAX_DS :
            speedR = +ds
            speedL = -ds
    
    robot.set_speed(speedL,speedR)

    # write a line of data in log file
    data.write("{},{},{},{},{},{},{},{},{},{}\n".format(step, K, T_I, T_D, PID_WALL_TARGET, pid.error, pid.P(), pid.I(), pid.D(), ds, speedL,speedR))
        
    step += 1
    
robot.clean_up()

