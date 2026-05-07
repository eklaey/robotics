from unifr_api_epuck import wrapper

MY_IP = '192.168.2.205' # modify the last number with the last 3 digits of the robot ID (on sticker)
robot = wrapper.get_robot(MY_IP)

counter = 0
cycle_length = 80 # 1 figure 8 cycle = forward_left + forward_right + backward_left + backward_right = 80 iterations

while robot.go_on() and counter < 400:
    counter += 1
    phase = counter % cycle_length

    if phase < cycle_length // 4:       # forward left
        robot.set_speed(2, 1)
    elif phase < cycle_length // 2:     # forward right
        robot.set_speed(1, 2)
    elif phase < 3 * cycle_length // 4: # backward left
        robot.set_speed(-2, -1)
    else:                               # backward right
        robot.set_speed(2, 2)

robot.clean_up()