# run this code to generate IR sensor data 
from unifr_api_epuck import wrapper

MY_IP = '192.168.2.205' # change robot number
MAX_STEPS = 1600        # used with a 40cm ruler, forward/backward
#600 for one way + stops for a bit on both sides
robot = wrapper.get_robot(MY_IP)

robot.init_sensors()
robot.calibrate_prox()

#open file for writing
data = open("IRsensors.csv", "w")

if data == None:
    print('Error opening data file!\n')
    quit

#write header in CSV file
data.write('step,')
for i in range(robot.PROX_SENSORS_COUNT):
    data.write('ps'+str(i)+',')
data.write('\n')

# wait 3 seconds before starting
robot.sleep(3)

for step in range(MAX_STEPS):
    if step < 600:
        robot.set_speed(2)
    elif step<800 or step>1400:
        robot.set_speed(0)
    else:
        robot.set_speed(-2)
    robot.go_on()
    ps = robot.get_calibrate_prox()
    #ps = robot.get_prox() # uncomment if analyzing uncalibrated sensor data

    #write a line of data 
    data.write(str(step)+',')
    for v in ps:
        data.write(str(v)+',')
    data.write('\n')
    
data.close()

robot.clean_up()

