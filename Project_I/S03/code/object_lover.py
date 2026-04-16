from unifr_api_epuck import wrapper
import os
import numpy as np

# Functions
def clamp(value, speed):
    minimum = -speed
    maximum = speed
    # Ensures motor speed never goes above MAX_SPEED
    return max(min(value, maximum), minimum)

def find_best_red_object(detections):
    red_objects = [d for d in detections if d.label == "Red Block"]
    if not red_objects:
        return None
    return max(red_objects, key=lambda d: d.confidence)

def find_best_red_color(color_detections):
    red_colors = [d for d in color_detections if d.label == "Red"]
    if not red_colors:
        return None
    return max(red_colors, key=lambda d: d.area)

# Main Object LOVER Program
MY_IP = '192.168.2.208'
robot = wrapper.get_robot(MY_IP)

# Initialization of sensors/model and parameters
robot.enable_all_led()
robot.sleep(3)

robot.init_sensors()
robot.initiate_model()
os.makedirs("./img", exist_ok=True)
robot.init_camera("./img")

robot.disable_all_led()

target = None
TARGET_DISTANCE = 135   # 10 cm
MAX_SPEED = 2.0

ANGULAR_GAIN = 0.5
DISTANCE_GAIN = 4    
THRESHOLD = 0.25  # Threshold for deciding when to switch between angular and distance control

OBJECT_DETECTION = True
COLOR_DETECTION = True

while robot.go_on():
    img = np.array(robot.get_camera())  
    camera_center = img.shape[2] / 2 # img shape is (channels, height, width), center is width/2
    
    # Object and color detection - determination of target object to follow (red block or red color patch)
    target = None
    if OBJECT_DETECTION:
        detections = robot.get_detection(img)
        target = find_best_red_object(detections)
        
    if COLOR_DETECTION and target is None:
        color_detections = robot.get_colordetection(img)
        target = find_best_red_color(color_detections)
    
    if target is None:
        robot.set_speed(0, 0)  # Stop if no target detected
        continue

    # ds calculation using Braitenberg formula: ds = -(error/threshold) * gain
    # ds for keeping target centered (angular control)
    x_error = (target.x_center - camera_center) / camera_center
    angular_ds = ANGULAR_GAIN * x_error

    # ds for keeping target at desired distance (linear control)
    # Use TOF for distance control as prox sensors are unreliable at TARGET_DISTANCE
    tof_distance = robot.get_tof()
    distance_error = (tof_distance - TARGET_DISTANCE) / TARGET_DISTANCE
    distance_ds = DISTANCE_GAIN * distance_error

    if np.abs(x_error) < THRESHOLD and np.abs(distance_error) > THRESHOLD:
        # If target is centered, use distance control to move forward/backward with angular correction
        left_speed = clamp(distance_ds + DISTANCE_GAIN * angular_ds, MAX_SPEED)
        right_speed = clamp(distance_ds - DISTANCE_GAIN * angular_ds, MAX_SPEED)
    else:
        # If target is not centered, use angular control to rotate towards it with no distance correction
        left_speed = clamp(angular_ds, MAX_SPEED)
        right_speed = clamp(-angular_ds, MAX_SPEED)
        
    robot.set_speed(left_speed, right_speed)

robot.clean_up()