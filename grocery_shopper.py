"""grocery controller."""

from controller import Robot, Camera, Display
import math
import numpy as np

#Initialization
print("=== Initializing Grocery Shopper...")
#Consts
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12
LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint",
              "gripper_left_finger_joint","gripper_right_finger_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.35, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf',0.045,0.045)

robot_parts={}
for i, part_name in enumerate(part_names):
    robot_parts[part_name]=robot.getDevice(part_name)
    robot_parts[part_name].setPosition(float(target_pos[i]))
    robot_parts[part_name].setVelocity(robot_parts[part_name].getMaxVelocity() / 2.0)

# Enable gripper encoders (position sensors)
left_gripper_enc=robot.getDevice("gripper_left_finger_joint_sensor")
right_gripper_enc=robot.getDevice("gripper_right_finger_joint_sensor")
left_gripper_enc.enable(timestep)
right_gripper_enc.enable(timestep)

# Enable Camera
camera = robot.getDevice('camera')
camera.enable(timestep)
#camera.recognitionEnable(timestep)

# Enable GPS and compass localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Enable LiDAR
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Enable display
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None
goal_waypoints = []





# ------------------------------------------------------------------
# Helper Functions

def is_new_waypoint(cx, cy, waypoints, threshold=5):
    for gx, gy in waypoints:
        if abs(gx - cx) <= threshold and abs(gy - cy) <= threshold:
            return False
    return True

def get_yellow_blob(camera):
    global last_detected_blob, reset_counter
    
    width = camera.getWidth()
    height = camera.getHeight()
    image = camera.getImage()
    
    yellow_pixels = []

    for x in range(width):
        for y in range(height):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)

            if r > 180 and g > 180 and b < 80 and abs(r - g) < 50:
                yellow_pixels.append((x, y))
    
    pixel_count = len(yellow_pixels)
    print(f"Yellow pixel count: {pixel_count}")
    
    if len(yellow_pixels) < 15:
        return None
    
    # Bounding box: min/max x/y  
    xs = [p[0] for p in yellow_pixels]  
    ys = [p[1] for p in yellow_pixels]  
    min_x, max_x = min(xs), max(xs)     
    min_y, max_y = min(ys), max(ys)    
    box_width = max_x - min_x           
    box_height = max_y - min_y         

    # Filter by size  
    # if box_width < 6 or box_height < 6:  
        # return None           
    # if box_width > width // 2 or box_height > height // 2:  
        # return None  

    # Filter by shape 
    # aspect_ratio = box_width / (box_height + 1e-5) 
    # if aspect_ratio < 0.5 or aspect_ratio > 1.5:  
        # return None 
 
    # if min_y < int(height * 0.3):  
        # return None  

    cx = int(np.mean(xs))  
    cy = int(np.mean(ys))  
    
    return (cx, cy)


gripper_status="closed"

# Main Loop
while robot.step(timestep) != -1:
    
    robot_parts["wheel_left_joint"].setVelocity(vL)
    robot_parts["wheel_right_joint"].setVelocity(vR)
    
    if(gripper_status=="open"):
        # Close gripper, note that this takes multiple time steps...
        robot_parts["gripper_left_finger_joint"].setPosition(0)
        robot_parts["gripper_right_finger_joint"].setPosition(0)
        if right_gripper_enc.getValue()<=0.005:
            gripper_status="closed"
    else:
        # Open gripper
        robot_parts["gripper_left_finger_joint"].setPosition(0.045)
        robot_parts["gripper_right_finger_joint"].setPosition(0.045)
        if left_gripper_enc.getValue()>=0.044:
            gripper_status="open"
            
    blob_center = get_yellow_blob(camera)
    
    if blob_center:
        cx, cy = blob_center
        print(f"Yellow cube detected at (x={cx}, y={cy}) in the camera frame.")
        
        if is_new_waypoint(cx, cy, goal_waypoints):
            goal_waypoints.append((cx, cy))
            
        image_center = camera.getWidth() / 2
        error_x = cx - image_center
        CAMERA_FOV = math.radians(1.57)  
        angle = (error_x / camera.getWidth()) * CAMERA_FOV
        
        Kp = 0.002
        turn_speed = Kp * error_x
        
        vL = 0.3 * MAX_SPEED - turn_speed
        vR = 0.3 * MAX_SPEED + turn_speed

    else:
        print("No yellow cube detected.")
        # Stop or spin to search
        vL = 0.2 *MAX_SPEED
        vR = 0.2 * MAX_SPEED
            
    
