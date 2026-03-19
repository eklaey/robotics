# run the code to generate IR sensor data 
from unifr_api_epuck import wrapper

MY_IP = '192.168.2.206'
MAX_STEPS = 300

# Behavior parameters for tuning robotic "love" and "exploration" dynamics
G_THRESHOLD = 10


# Sensor definition
G_LEFT = 0
G_MIDDLE = 1
G_RIGHT = 2


# State definitions
# 1 1 0 CORRECT
# 0 0 0 NO INFO -> go straight
# 1 0 0 -> go left
# 


# Check if sensor is on the line
def on_line(gs):
    if gs > G_THRESHOLD:
        return 1
    else:
        return 0

# Storage of recent ground values for smoothing; will trim to last store only 10 values manually
ground_array = []

# Initial conditions for state machine
speed_left = 2
speed_right = 2

# Initialize robot
# robot.init_sensors() 
robot.init_ground()
robot.sleep(5)

robot = wrapper.get_robot(MY_IP)


for step in range(MAX_STEPS):
    robot.go_on()
    gs = robot.get_ground()
    
    # should make an average ? 

    match [on_line(gs[G_LEFT]), on_line(gs[G_MIDDLE]), on_line(gs[G_RIGHT])]:
        case [1, 1, 0]:
            speed_left, speed_right = 2, 2
        case [1, 0, 0]:
            speed_left, speed_right = 2, 0
        case [1, 1, 1]:
            speed_left, speed_right = 0, 2
        case [0, 1, 1] | [0, 0, 1]:
            speed_left, speed_right = 0, 2
        case _:
            speed_left, speed_right = 2, 2

    robot.set_speed(speed_left, speed_right)
    

robot.clean_up()
