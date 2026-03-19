# run the code to generate IR sensor data 
from unifr_api_epuck import wrapper

MY_IP = '192.168.2.206' 
MAX_STEPS = 500     # Increased to ensure it crosses the full setup

robot = wrapper.get_robot(MY_IP)

# Initialize ground sensors
robot.init_ground()

# Open file for writing
data = open("Gsensors.csv", "w")

if data == None:
    print('Error opening data file!\n')
    quit

# Write header in CSV file
data.write('step,')
for i in range(robot.GROUND_SENSORS_COUNT):
    data.write('gs'+str(i)+',')
data.write('\n')

# Wait 3 seconds before starting to give you time to place it
print("Starting in 3 seconds...")
robot.sleep(3)

for step in range(MAX_STEPS):
    # This allows the robot to process controller events
    if not robot.go_on():
        break
    
    # --- MOVEMENT COMMAND ---
    # Set a slow constant speed (2 rad/s) to get high-resolution data
    robot.set_speed(2, 2) 
    
    # Read ground sensors
    gs = robot.get_ground()
    
    # Write a line of data
    data.write(str(step)+',')
    for v in gs:
        data.write(str(v)+',')
    data.write('\n')
    
# STOP the robot after the loop finishes
robot.set_speed(0, 0)
data.close()
robot.clean_up() #
print("Data collection complete. File saved as Gsensors.csv")