from unifr_api_epuck import wrapper

MY_IP = '192.168.2.202' # change last number to match with last 3 digits of robot ID (on sticker)
robot = wrapper.get_robot(MY_IP)

while robot.go_on(): # will not stop
    pass
    
robot.clean_up()
