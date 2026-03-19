# run the script to generate additional sensor data 
from unifr_api_epuck import wrapper

MY_IP = '172.20.10.14'
MAX_STEPS = 500
STATIONARY_STEPS = 100

STOPPING_DISTANCE = 5

robot = wrapper.get_robot(MY_IP)

#open file for writing
data = open("../recordings/sensors.csv", "w")

if data == None:
    print('Error opening data file!\n')
    quit

#write header in CSV file
data.write('step,')
data.write('tof,')
data.write('ps0,ps1,ps2,ps3,ps4,ps5,ps6,ps7')
data.write('\n')

# Sensor callibration before running test
robot.init_sensors()
robot.calibrate_prox()

# wait 10 seconds before starting
robot.sleep(10)

for step in range(MAX_STEPS):
    robot.go_on()
    
    if step < STATIONARY_STEPS or robot.get_tof() < STOPPING_DISTANCE:
        # Measures baseline TOF rate at set distance
        robot.set_speed(0, 0)
    else:
        # Measures TOF changes when robot approches wall
        # at constant speed
        robot.set_speed(2, 2)
    
    #write a line of data 
    data.write(str(step)+',')

    data.write(str(robot.get_tof())+',')
    
    ps = robot.get_calibrate_prox()
    for val in ps:
        data.write(str(val)+',')
  
    data.write('\n')
    
robot.set_speed(0, 0)
data.close()
robot.clean_up()
print("Data collection complete. File saved as sensors.csv")