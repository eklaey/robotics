# run the script to generate additional sensor data 
from unifr_api_epuck import wrapper

MY_IP = '172.20.10.14'
MAX_STEPS = 500
STATIONARY_STEPS = 100

STOPPING_DISTANCE = 5

robot = wrapper.get_robot(MY_IP)

#open file for writing
data = open("../recordings/TOFsensors.csv", "w")

if data == None:
    print('Error opening data file!\n')
    quit

#write header in CSV file
data.write('step,')
data.write('tof,')
data.write('accX,accY,accZ,')
data.write('acc,incl,orient,')
data.write('roll,pitch,')
data.write('gyroX,gyroY,gyroZ,')
data.write('mic0,mic1,mic2,mic3,')
data.write('\n')

# wait 3 seconds before starting
robot.sleep(3)

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

    xyz = robot.get_accelerometer_axes()
    for v in xyz:
        data.write(str(v)+',')

    data.write(str(robot.get_acceleration())+',')
    data.write(str(robot.get_inclination())+',')
    data.write(str(robot.get_orientation())+',')
    data.write(str(robot.get_roll())+',')
    data.write(str(robot.get_pitch())+',')

    gyro = robot.get_gyro_axes()
    for g in gyro:
        data.write(str(g)+',')
    
    mic = robot.get_microphones()
    for v in mic:
        data.write(str(v)+',')

    data.write('\n')
    
robot.set_speed(0, 0)
data.close()
robot.clean_up()