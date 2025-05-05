
"""grocery controller."""

from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard, Display
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve 
import random
import time
import cv2

'''
To complete this project we used a variety of npy files and different modes:
For all modes the robot starts at coordinates: -14.2, -5.77 with the Lidar Sensor at: 0.97
Autonomous mode: This is the autonomous mapping mode and must be used first: creates
This is our autonomous mapping mode. Saves the coordinates of block (goal_waypoints.npy)
and saves the map (auto_map.npy)
Planner Mode: This is the planning mode and must be used second. It uses RRT algorithm to get
paths to all of the goal points/way points (block points) in our map. Creates the grid of all
waypoints (2D array) (current_path_grid.npy) and saves the convolved map (convolved_auto_map.npy)
Odometry mode: This is step 3 of our project. This is where the robot maps to the waypoints and
then moves to the position it needs to be in to pickup the block and then we manually pick up and
place our block into the robots shopping cart.
'''

#Initialization
print("=== Initializing Grocery Shopper...")
#Consts
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 14
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

# 

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.35, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf',0.045,0.045)

robot_parts=[]
for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)





# Enable gripper encoders (position sensors)
left_gripper_enc=robot.getDevice("gripper_left_finger_joint_sensor")
right_gripper_enc=robot.getDevice("gripper_right_finger_joint_sensor")
left_gripper_enc.enable(timestep)
right_gripper_enc.enable(timestep)

# Enable Camera
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# Enable GPS and compass localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Enable LiDAR
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()
# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# Enable display
display = robot.getDevice("display")
# image = camera.getImage()

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

# Map Configuration
# Grid dimensions represent the physical size covered by the map (e.g., 30m x 16m)
MAP_WIDTH_METERS = 30.0
MAP_HEIGHT_METERS = 16.0
# Grid resolution (number of cells)
# Using 360x192 cells for a 30m x 16m map
GRID_WIDTH_CELLS = 360
GRID_HEIGHT_CELLS = 192
# Calculate cell size based on map dimensions and grid resolution
# Ensure cells are square for isotropic mapping
CELL_SIZE = MAP_WIDTH_METERS / GRID_WIDTH_CELLS # Should be 30/480 = 0.0625
# Verify if height calculation matches: MAP_HEIGHT_METERS / GRID_HEIGHT_CELLS = 16/256 = 0.0625. It does.
print(f"Map Cell Size: {CELL_SIZE:.4f} meters")

map = np.zeros((GRID_WIDTH_CELLS, GRID_HEIGHT_CELLS), dtype=np.float32)

SPEED = 5
# Initialize motors
left_motor = robot_parts[MOTOR_LEFT]
right_motor = robot_parts[MOTOR_RIGHT]

# ------------------------------------------------------------------
# Helper Functions

# We will say that (0,0) is the bottom left of where the robot begins on the wall with the table. 
# Modify world to grid so that you can account for negative and positive values and rectangle

# Convert world coordinates (meters, continuous) to grid cell indices (integers)
# Assumes world origin (0,0) maps to the center of the grid
def world_to_grid(x, y):
    # Calculate offset from the grid center
    grid_x = int(x / CELL_SIZE + GRID_WIDTH_CELLS / 2)
    grid_y = int(y / CELL_SIZE + GRID_HEIGHT_CELLS / 2)

    # Clamp indices to be within grid bounds
    grid_x = max(0, min(grid_x, GRID_WIDTH_CELLS - 1))
    grid_y = max(0, min(grid_y, GRID_HEIGHT_CELLS - 1))

    return grid_x, grid_y

# Convert grid cell indices back to world coordinates (center of the cell)

def grid_to_world(grid_x, grid_y):
    x = (grid_x - GRID_WIDTH_CELLS / 2 + 0.5) * CELL_SIZE
    y = (grid_y - GRID_HEIGHT_CELLS / 2 + 0.5) * CELL_SIZE 
    # print(x, y)
    return (x-0.2), (y - 0.3)

# Load map   
def load_map(filename):
    loaded_map = np.load(filename).astype(np.float32)
    plt.imshow(loaded_map)
    plt.show()
    return loaded_map

# # Saves a convolved
def convolve_map(filename):
    smear = np.ones((18,18))
    plan_map = load_map(filename)
    print("in convolve map")
    print("plan_map shape:", plan_map.shape)
    print("smear shape:", smear.shape)
    plan_map = convolve(plan_map, smear)
    plan_map = np.clip(plan_map, 0, 1) # takes all values if below zero zero, above 1 1
    plan_map = np.multiply(plan_map>0.5,1)
    np.save("convolved_auto_map.npy", plan_map)
    # for debugging concerns load the map
    # plan_map_shown = np.load("convolved_map.npy")
    # plt.imshow(plan_map_shown)
    # plt.show()
    return
    
def grey_scale(value):
    """ Scale map value [0, 1] to grayscale color """
    #  # Clamp value just in case
    # value = max(0.0, min(1.0, value))
    gray = int(value * 255)
    return (gray << 16) + (gray << 8) + gray
    
gripper_status="closed"

# --------------------------------------------------------------------------------------------
# rrt implmentation functions
class Node:
    """Node for RRT Algorithm."""
    def __init__(self, pt, parent=None):
        self.point = pt # np.array([x, y])
        self.parent = parent # Parent node
        self.path_from_parent = [] # Optional: List of points along the edge from parent

def get_nearest_vertex(node_list, q_point):
    """Finds the node in node_list closest to q_point."""
    shortest_distance = float('inf')
    nearest_vertex = None
    for node in node_list:
        distance = np.linalg.norm(np.array(node.point) - np.array(q_point))
       
        if distance < shortest_distance:
            shortest_distance = distance
            nearest_vertex = node

    return nearest_vertex


def steer(from_point, to_point, delta_q):
    """
    Steers from from_point towards to_point by taking integer steps (grid moves),
    up to a maximum of delta_q steps.
    Returns the list of points (path) and the final point reached.
    """
    
    from_point = np.array(from_point, dtype=int)
    to_point = np.array(to_point, dtype=int)
    
    current = from_point.copy()
    path = [current.copy()]
    
    steps_taken = 0
    
    while not np.array_equal(current, to_point) and steps_taken < delta_q:
        move = np.sign(to_point - current)  # move one step in each direction towards target
        current += move
        path.append(current.copy())
        steps_taken += 1
    return np.array(path), current
    
    
# Makes sure you can move to where you are trying to
def state_is_valid(state, bounds, grid):
    """
    Checks if a state (x, y point) is valid.
    Checks grid space to see if it is greater than zero, if it is we know space is taken by wall
    Currently only checks against bounds.

    """
   
    if not ((bounds[0, 0] <= state[0]) and (state[0] <= bounds[0, 1]) and
            (bounds[1, 0] <= state[1]) and (state[1] <= bounds[1, 1])):
 
        return False
    if grid[state[0]][state[1]] > 0: 
     
        return False

    return True 
    
    
# Checks if you can go to the edge
def edge_is_valid(path_segment, bounds, grid):
    """Checks if all points along a path segment are valid."""
    for point in path_segment:
        if not state_is_valid(point, bounds, grid):
            return False
    return True

# Gives the bounds in grid coordinates
ARENA_BOUNDS = np.array([
    [0, 360], # X-axis bounds #was -10, 10
    [0, 192]  # Y-axis bounds #was -10,10
])



# --- RRT Algorithm ---
def rrt(grid, state_bounds, state_valid_func, edge_valid_func, starting_point, goal_point, k, delta_q, goal_bias=0.05, goal_tolerance=0.5):
    """
    RRT Algorithm Implementation.

    @param state_bounds: matrix of min/max values for each dimension
    @param state_valid_func: function(state, bounds) -> bool
    @param edge_valid_func: function(path_segment, bounds) -> bool
    @param starting_point: np.array([x, y])
    @param goal_point: np.array([x, y])
    @param k: Max number of iterations
    @param delta_q: Max distance for steer function
    @param goal_bias: Probability of sampling the goal point directly
    @param goal_tolerance: Distance threshold to consider the goal reached
    @returns List of RRT graph nodes, or None if no path found in k iterations
    """
    node_list = []
    start_node = Node(np.array(starting_point), parent=None)
   
    if grid[goal_point[0]][goal_point[1]] > 0:
        print("goaal point in convolve")

    node_list.append(start_node)
    goal_point = np.array(goal_point) # Ensure goal is numpy array

    print(f"RRT: Start={starting_point}, Goal={goal_point}, K={k}, delta_q={delta_q}")

    for i in range(k):
        # 1. Sample Point (with goal bias)
        if random.random() < goal_bias:
            q_rand = goal_point
            # print(f"Iter {i}: Sampling Goal")
        else:
            # Sample randomly within bounds
            q_rand_x = random.randint(state_bounds[0, 0], state_bounds[0, 1])
            q_rand_y = random.randint(state_bounds[1, 0], state_bounds[1, 1])
            q_rand = np.array([q_rand_x, q_rand_y])

        # 2. Find Nearest Node
        nearest_node = get_nearest_vertex(node_list, q_rand)
        
        # 3. Steer from Nearest towards Sampled
        path_segment, q_new_point = steer(nearest_node.point, q_rand, delta_q)

        # 4. Check Validity of Edge and New Point
        if state_valid_func(q_new_point, state_bounds, grid) and edge_valid_func(path_segment, state_bounds, grid):
            #  print(f"Iter {i}: Edge is valid.")
             # 5. Add New Node
             new_node = Node(q_new_point, parent=nearest_node)
             new_node.path_from_parent = path_segment # Store the edge path
             node_list.append(new_node)

             # 6. Check if Goal Reached
             dist_to_goal = np.linalg.norm(q_new_point - goal_point)
             if dist_to_goal <= goal_tolerance:
                 print(f"RRT: Goal reached at iteration {i}!")
                 # Add final node representing the goal
                 goal_node = Node(goal_point, parent=new_node)
                 # Optional: steer directly to goal if possible
                 final_segment, _ = steer(new_node.point, goal_point, delta_q)
                 if edge_valid_func(final_segment, state_bounds, grid):
                    goal_node.path_from_parent = final_segment
                    node_list.append(goal_node)
                    print("RRT: Final edge to precise goal added.")
                 else:
                     # If direct path is blocked, the previously added node is the closest we got
                     print("RRT: Direct final edge to goal blocked, using node near goal.")
                     goal_node = new_node # Use the node that was close enough

                 return node_list, goal_node # Return all nodes and the final node

    print(f"RRT: Failed to reach goal within {k} iterations.")
    # Find node closest to goal if exact goal wasn't reached
    final_node = get_nearest_vertex(node_list, goal_point)
    return node_list, final_node # Return all nodes and the node closest to the goal

# Goes back and builds the path we just took to get our waypoints
def reconstruct_path(goal_node):
    """Backtracks from the goal node to the start node to get the path."""
    path = []
    current = goal_node
    while current is not None:
        path.append(current.point)
        current = current.parent
    path.reverse() # Reverse to get path from start to goal
    # Convert to list of tuples if preferred for waypoint following
    waypoints = [tuple(p) for p in path]
    return waypoints


# --- Robot Control Functions ---
def calculate_errors(pos_x, pos_y, pos_theta, goal_x, goal_y):
    """Calculates position and bearing error."""
    pos_err = math.sqrt((goal_x - pos_x) ** 2 + (goal_y - pos_y) ** 2)
    goal_direction = math.atan2(goal_y - pos_y, goal_x - pos_x)
    bearing_err = goal_direction - pos_theta
    # Normalize bearing error to [-pi, pi]
    bearing_err = (bearing_err + math.pi) % (2 * math.pi) - math.pi
    return pos_err, bearing_err

# --------------------------------------------------------------------------------------------
# ODOMETRY 
# --- Robot Control Functions ---

# This is our odometry function that allows us to see our errors
def calculate_errors(pos_x, pos_y, pos_theta, goal_x, goal_y):
    """Calculates distance and bearing error to a world goal."""
    pos_err = math.sqrt((goal_x - pos_x) ** 2 + (goal_y - pos_y) ** 2)
    goal_direction = math.atan2(goal_y - pos_y, goal_x - pos_x) # Target angle in world frame
    bearing_err = goal_direction - pos_theta # Difference between target angle and robot's current angle

    # Normalize bearing error to [-pi, pi]
    while bearing_err > math.pi: bearing_err -= 2 * math.pi
    while bearing_err < -math.pi: bearing_err += 2 * math.pi

    return pos_err, bearing_err

# Simple P Controller Gains
KP_ANGLE = 2.5 # Proportional gain for turning speed based on bearing error
KP_POS = 0.0   # Proportional gain for forward speed (set to 0 to turn first)
FORWARD_SPEED_WHEN_ALIGNED = 2.5 # Base forward speed when reasonably aligned [rad/s]
BEARING_THRESHOLD_FOR_FORWARD = math.radians(25) # Allow forward motion if bearing error is within this threshold [radians]
WAYPOINT_REACHED_THRESHOLD = 0.4 # How close the robot needs to be to a waypoint (in meters)

# --------------------------------------------------------------------------------------------
# COLOR VISION 


# This checks that the new color we have spotted is not to close to an already existing waypoint we have looked at
def is_new_waypoint(cx, cy, waypoints, threshold=0.5):
    for gx, gy in waypoints:
        if abs(gx - cx) <= threshold and abs(gy - cy) <= threshold:
            return False
    return True

# Uses camera to detect yellow blobs
def get_yellow_blob(camera):
    
    width = camera.getWidth()
    height = camera.getHeight()
    image = camera.getImage()
    
    # Convert image to numpy array
    img = np.zeros((height, width, 3), dtype=np.uint8)
    for x in range(width):
        for y in range(height):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)
            # OpenCV uses bgr not rgb!
            img[y, x] = [b, g, r]  
    
    # Convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])  #[20, 80, 80]
    upper_yellow = np.array([30, 255, 255]) #[32, 255, 255]
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # mask of yellow in black and white bg
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blob_centroids = []
    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        
        if area > 25:  
            # moment of contour (finding the center)
            M = cv2.moments(cnt)
            
            # if area is not zero (prevent error)
            if M['m00'] != 0:
                # finding the centroid of the contour
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # find contours of the bounding box
                x, y, w, h = cv2.boundingRect(cnt)
                
                aspect_ratio = w / float(h + 1e-5)
                
                if aspect_ratio < 0.75 or aspect_ratio > 1.25:
                    continue           
                    
                blob_centroids.append((cx, cy))
                
                # optional filtering
                # aspect_ratio = w / float(h + 1e-5)
                # if aspect_ratio < 0.7 or aspect_ratio > 1.5:
                    # continue   
                # if w < 5 or h < 0.5:
                    # continue
                    
                # Draw yellow rectangle over detected blob
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
                # Draw centroid
                cv2.circle(img, (cx, cy), 2, (0, 0, 255), -1)
                # Label it
                cv2.putText(img, f"Blob {i+1}", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    
    cv2.imshow("Yellow Mask", mask)
    cv2.imshow("Debug view", img)
    cv2.waitKey(1)
        
    return blob_centroids

# Uses camera to detect green blobs
def get_green_blob(camera):
    
    width = camera.getWidth()
    height = camera.getHeight()
    image = camera.getImage()
    
    # Convert image to numpy array
    img = np.zeros((height, width, 3), dtype=np.uint8)
    for x in range(width):
        for y in range(height):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)
            # OpenCV uses bgr not rgb!
            img[y, x] = [b, g, r]  
    
    # Convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([40, 50, 50]) 
    upper_yellow = np.array([85, 255, 255]) #[32, 255, 255]
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # mask of yellow in black and white bg
    # retrieve contours, retrieve outermost contour (4 point edges of rectangle)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    blob_centroids = []
    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        
        if area > 25:  
            # moment of contour (finding the center)
            M = cv2.moments(cnt)
            
            # if area is not zero (prevent error)
            if M['m00'] != 0:
                # finding the centroid of the contour
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # find contours of the bounding box
                x, y, w, h = cv2.boundingRect(cnt)
                
                aspect_ratio = w / float(h + 1e-5)
                
                if aspect_ratio < 0.75 or aspect_ratio > 1.25:
                    continue           
                    
                blob_centroids.append((cx, cy))
                
                # optional filters 
                # if w < 5 or h < 0.5:
                    # continue
                    
                # Draw yellow rectangle over detected blob
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Draw centroid
                cv2.circle(img, (cx, cy), 2, (0, 0, 255), -1)
                # Label it
                cv2.putText(img, f"Green Blob {i+1}", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
    cv2.imshow("Green Mask", mask)
    cv2.imshow("Debug Green view", img)
    cv2.waitKey(1)
        
    return blob_centroids

# --------------------------------------------------------------------------------------------
# ROBOT MANIPULATION 
# robot_position = False

def robot_pickup(index, pose_x, pose_y, pose_theta):

    BEARING_THRESHOLD_FOR_FORWARD_2= math.radians(10)
    WAYPOINT_REACHED_THRESHOLD_2 = 0.1
    
    robot_finished = False
    robot_position = False
    
    while robot_finished == False: 
        robot.step(timestep)
        
        position = gps.getValues()
        orientation = compass.getValues()
        pose_x = position[0]
        pose_y = position[1]
        pose_theta = math.atan2(orientation[0], orientation[1])
    
        print(index, "in while loop")
        
        if index == 0: 
            goal_x = -1.50241 
            goal_y = -5.19031 
            
        if index == 1: 
            goal_x = 5.23
            goal_y = -2.759
            
        if index == 2: 
            goal_x = 2.394 
            goal_y = -2.74179

        if index == 3: 
            goal_x = -2.54
            goal_y = 1.1424  
            
        if index == 4: 
            goal_x = -3.201
            goal_y = 1.17       
        
        if index == 5: 
            goal_x = -0.81
            goal_y = 1.33         
            
        if index == 6: 
            goal_x = 0.908
            goal_y = 1.246           

        if index == 7: 
            goal_x = 3.05 
            goal_y = 2.765           
        
        if index == 8: 
            goal_x = 5.68
            goal_y = 6.34 

        if index == 9: 
            goal_x = 3.37
            goal_y = 6.44

        pos_err, bearing_err = calculate_errors(pose_x, pose_y, pose_theta, goal_x, goal_y)

        if abs(bearing_err) > BEARING_THRESHOLD_FOR_FORWARD_2:

                    vL = -KP_ANGLE * bearing_err
                    vR = KP_ANGLE * bearing_err
                    
                    vL = max(-MAX_SPEED, min(MAX_SPEED, vL))
                    vR = max(-MAX_SPEED, min(MAX_SPEED, vR))
                    
                    print(f"Turning: BearErr={math.degrees(bearing_err):.1f}deg | vL={vL:.2f}, vR={vR:.2f}")
        else:
                    forward_component = FORWARD_SPEED_WHEN_ALIGNED 
                    turn_component = KP_ANGLE * bearing_err 

                    vL = forward_component - turn_component
                    vR = forward_component + turn_component

                    vL = max(-MAX_SPEED, min(MAX_SPEED, vL))
                    vR = max(-MAX_SPEED, min(MAX_SPEED, vR))
                    
                    print(f"Moving: PosErr={pos_err:.2f}m, BearErr={math.degrees(bearing_err):.1f}deg | vL={vL:.2f}, vR={vR:.2f}")

        if pos_err < WAYPOINT_REACHED_THRESHOLD_2:
            robot_position = True 
        
        if robot_position == True: 
            robot_finished = True

        robot_parts[MOTOR_LEFT].setVelocity(vL)
        robot_parts[MOTOR_RIGHT].setVelocity(vR)
            
    return 

# --------------------------------------------------------------------------------------------

ALL_GOAL_POINTS_WORLD = [
    #(3.77,-5.39),
    # (9.5, -1.65), 
    # (9.5, -1.55), 
    # (9.5, -1.45),
    # (9.5, -1.35),
    # (9.5, -1.25),
    # (2.81, 2.55),
    # (2.63, 6.31)
    # (12.9,-4.14),
    # (1, -2.16),
    # (-9, -2.15),
    # (3.06, 2.04)
]

# mode = 'manual' #manual mode
# mode = 'autonomous' #autonomous mode
# mode = 'planner'
# mode = 'planner_auto'
mode = 'odometry'

# If mode is odometry we are loading the current 2D array waypoints, the rest of the code will be done in while loop

if mode == 'odometry':
    current_path_grid = np.load("current_path_grid.npy", allow_pickle=True)
    current_waypoint_index = 0

elif mode == 'planner':
        print("in planner mode")
        convolve_map("map.npy")
        print("out of convole map")
        convolved_map = load_map("convolved_map.npy")
        print("loaded convolved map")
      
# If mode is planner auto we need to convolve the map and run the RRT algorithm for each of our goal waypoints
# we create a path from waypoint to waypoint, save the path and then print it on a map

elif mode == "planner_auto":
   
        while robot.step(timestep) != -1:
            print("in timestep")
            left_motor.setVelocity(SPEED)
            right_motor.setVelocity(SPEED)
            for _ in range(125):
                robot.step(timestep)
           
            break
          

        # 1. Get Initial Robot State (Wait for valid GPS)
        print("Waiting for valid GPS signal...")
        pose_x, pose_y, pose_theta = 0, 0, 0
        while robot.step(timestep) != -1:
            position = gps.getValues()
            if not any(math.isnan(p) for p in position) and abs(position[0]) > 1e-6 : # Check for valid (non-zero, non-NaN) position
                pose_x = position[0]
                pose_y = position[1]
                orientation = compass.getValues()
                if not any(math.isnan(o) for o in orientation):
                    pose_theta = np.arctan2(orientation[0], orientation[1]) # Webots compass X is often North, Y is East
                    print(f"Initial Pose: x={pose_x:.2f}, y={pose_y:.2f}, theta={math.degrees(pose_theta):.1f} deg")
                    break # Got valid initial pose
            print("Still waiting for GPS...")

       
        convolve_map("auto_map.npy")
        convolved_map = load_map("convolved_auto_map.npy")
      
        # Get starting information for RRT Algorithm to run
        config_space = convolved_map #np array
        pose_x = gps.getValues()[0]
        pose_y = gps.getValues()[1]
        start_x, start_y = world_to_grid(pose_x, pose_y)
        start = np.array([start_x, start_y])
        # RRT Parameters (adjust as needed)
        K_ITERATIONS = 3000   # Max iterations
        DELTA_Q = 4      # Max step size (meters) #was 0.5
        GOAL_BIAS = 0.10    # 10% chance to sample goal
        GOAL_TOLERANCE = 2 # How close to goal is considered "reached" (meters)

        goal_point = np.load("goal_waypoints.npy")
        goal_point = [(x1+1, y1 + 0.2) for (x1, y1) in goal_point]
        
        ALL_GOAL_POINTS_WORLD = goal_point
        
        # Check waypoints if they aren't working 
        for leng in ALL_GOAL_POINTS_WORLD:
            print(leng)

        # 2. Define Goal
        
        ALL_GOAL_POINTS_PATH = []
        for index in ALL_GOAL_POINTS_WORLD:
            end_x, end_y = world_to_grid(index[0], index[1])
            end = np.array([end_x, end_y ]) # waypoints

            # 3. Run RRT to Generate Path
            print("Running RRT...")
            start_time = time.time()
    
            all_nodes, final_node = rrt(
                grid = config_space,
                state_bounds=ARENA_BOUNDS,
                state_valid_func=state_is_valid,
                edge_valid_func=edge_is_valid,
                starting_point=start,
                goal_point=end,
                k=K_ITERATIONS,
                delta_q=DELTA_Q,
                goal_bias=GOAL_BIAS,
                goal_tolerance=GOAL_TOLERANCE
            )
            end_time = time.time()
            print(f"RRT calculation finished in {end_time - start_time:.2f} seconds.")

            # 4. Reconstruct Path
            if final_node and np.linalg.norm(final_node.point - end) <= GOAL_TOLERANCE:
                print("Reconstructing path...")
                waypoints = reconstruct_path(final_node)
                print(f"Path found with {len(waypoints)} waypoints:")
                for i, wp in enumerate(waypoints):
                    print(f"  {i}: ({wp[0]:.2f}, {wp[1]:.2f})")
            else:
                print("RRT did not find a path to the goal or the closest node is too far.")
                
                if final_node:
                    waypoints = reconstruct_path(final_node)
                    print(f"Using path to closest node found ({len(waypoints)} waypoints):")
                    for i, wp in enumerate(waypoints):
                        print(f"  {i}: ({wp[0]:.2f}, {wp[1]:.2f})")
                else:
                    print("RRT failed completely. Exiting.")
                    waypoints = [] # No path

            # Show the path on our convolved map
            path_np = np.array(waypoints)
            plt.imshow(convolved_map)
            plt.plot(path_np[:, 1], path_np[:, 0], c='red', linewidth=2) 
            plt.show()
            ALL_GOAL_POINTS_PATH.append(path_np)
            
            # Update the start coordinates so our robot has a new start
            start = end
       
        np.save("current_path_grid.npy", np.array(ALL_GOAL_POINTS_PATH, dtype=object))

# various variables used in our function
turning = False
turn_counter = 0
TURN_DURATION = 840  # number of steps to finish the turn
break_while = False
TURN_SPEED = 2 # speed of robot while turning in autonomous mode

left_motor.setVelocity(0)
right_motor.setVelocity(0)
goal_bool = False
index = 0 

goal_waypoints = []

arm_up = False
robot_state = 'goal_not'
robot_position = False
# Alternation variable for autonomous mapping, switches between true and false
last_turn_left = False

# Main Loop
while robot.step(timestep) != -1:
    if arm_up == False:
        for _ in range(125):
                robot.step(timestep)
        arm_up = True
    # print(arm_up)
    if break_while == True: # or goal_bool == True: 
        break
        ################ v [Begin] Do not modify v ##################
    # Ground truth pose
    # print(type(map))
    
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]
    
    n = compass.getValues()
    rad = math.atan2(n[0], n[1])
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
    # ranges = lidar.getRangeImage()

    if mode == "manual" or mode == "autonomous":
        for i, rho in enumerate(lidar_sensor_readings):
            alpha = lidar_offsets[i]
        
            if rho > LIDAR_SENSOR_MAX_RANGE - 0.01:
                continue

            # The Webots coordinate system doesn't match the robot-centric axes we're used to
            rx = math.cos(alpha)*rho # detection of object in x in robot coordinates
            ry = -math.sin(alpha)*rho # detection of object in y in robot coordinates

            t = pose_theta
            # print(pose_theta)
            # + np.pi/2.
            # Convert detection from robot coordinates into world coordinates
            wx =  math.cos(t)*rx - math.sin(t)*ry + pose_x# detection of object in x in world coordinates
            wy =  math.sin(t)*rx + math.cos(t)*ry + pose_y # detection of objecct in y in world coordinates
    
            ################ ^ [End] Do not modify ^ ##################

            # print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))
            if wx >= 16:
                wx = 15.999
            if wy >= 12:
                wy = 11.999
            if rho < LIDAR_SENSOR_MAX_RANGE:
                # Part 1.3: visualize map gray values.
                grid_x,grid_y = world_to_grid(wx,wy)
                map[grid_x][grid_y] = min(map[grid_x][grid_y] + 5e-3, 1.0)
            
                display.setColor(grey_scale(map[grid_x][grid_y]))
                display.drawPixel(360-grid_x,grid_y)

            # Convert world coordinates (wx, wy) to map grid indices (grid_x, grid_y)
            grid_x, grid_y = world_to_grid(wx, wy)

            # Update the occupancy grid map at the detected point
            # Simple additive update: increase probability of occupancy, capped at 1.0
            # More advanced: Use log-odds update for better handling of uncertainty and free space.
            # Current simple method:
            current_val = map[grid_x, grid_y]
            # Increase occupancy probability, ensure it doesn't exceed 1.0
            # The increment value (e.g., 5e-3) affects how quickly cells become 'occupied'. Tune as needed.
            map[grid_x, grid_y] = min(current_val + 0.05, 1.0) # Increased increment slightly

        # Draw the robot's current position on the display
        robot_grid_x, robot_grid_y = world_to_grid(pose_x, pose_y)
        display.setColor(int(0xFF0000))  # Red color for the robot
        # Use the same coordinate mapping as the map drawing
        robot_display_x = GRID_WIDTH_CELLS - 1 - robot_grid_x # Flip X
        robot_display_y = robot_grid_y # Y as is
        # Draw a small cross or circle for the robot
        display.drawPixel(robot_display_x, robot_display_y)

        num_points = len(lidar_sensor_readings)
        front_distance = lidar_sensor_readings[num_points // 2]
    
    
    if mode == 'autonomous':
        
        blob_detected = get_yellow_blob(camera)
        green_blob_detected = get_green_blob(camera)

        if blob_detected:
            for cx, cy in blob_detected:
                # for later use
                image_center = camera.getWidth() // 2
                error = cx - image_center
                
                # store robot's current pos at the time of its yellow cube detection
                cube_x = gps.getValues()[0]
                cube_y = gps.getValues()[1]
            
                if is_new_waypoint(cube_x, cube_y, goal_waypoints):
                    goal_waypoints.append((cube_x, cube_y))
                    print(f"Saved goal waypoint at({cube_x:.2f}, {cube_y:.2f})")
                  
        elif green_blob_detected:
            for cx, cy in green_blob_detected:
                # for later use
                image_center = camera.getWidth() // 2
                error = cx - image_center
                
                # store robot's current pos at the time of its yellow cube detection
                cube_x = gps.getValues()[0]
                cube_y = gps.getValues()[1]
            
                if is_new_waypoint(cube_x, cube_y, goal_waypoints):
                    goal_waypoints.append((cube_x, cube_y))
                    print(f"Saved goal waypoint at({cube_x:.2f}, {cube_y:.2f})")
        
        else:
        
            front_distance = lidar_sensor_readings[len(lidar_sensor_readings) // 2]
            
            if front_distance < 0.3:
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                print(goal_waypoints)
                np.save("goal_waypoints.npy", goal_waypoints)
                filtered_map = np.multiply(map > 0.7, 1)
                np.save("auto_map.npy", filtered_map)
                print("Map saved")
                load_map("auto_map.npy")
                break_while = True
                continue
        
            if turning:
                if last_turn_left:
                    #print("Turning right")
                    left_motor.setVelocity(SPEED / 2)
                    right_motor.setVelocity(TURN_SPEED)
                else:
                    #print("Turning left")
                    left_motor.setVelocity(TURN_SPEED)
                    right_motor.setVelocity(SPEED / 2)
                turn_counter += 1
                if turn_counter >= TURN_DURATION:
                    turning = False
                    turn_counter = 0
                    last_turn_left = not last_turn_left
            else:
                if front_distance < 3:
                    turning = True
                    turn_counter = 0
                    if last_turn_left:
                        #print("Turning right two")
                        left_motor.setVelocity(SPEED / 2)
                        right_motor.setVelocity(TURN_SPEED)
                    else:
                        #print("Turning left two")
                        left_motor.setVelocity(TURN_SPEED)
                        right_motor.setVelocity(SPEED / 2)
                else:
                    left_motor.setVelocity(SPEED)
                    right_motor.setVelocity(SPEED) 

    elif mode == 'manual':
    
        key = keyboard.getKey()
        while keyboard.getKey() != -1:
            pass  
            
        print("Manual keys:")
        print(" - 1-7 = Raise joints")
        print(" - q-u = Lower joints")
        print(" - Z/X = Raise/Lower torso")
        print(" - D = Switch to odometry")
        print(" - O/C = Open/Close gripper")
                
        if key == keyboard.LEFT:
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):  
            vL = 0
            vR = 0
        elif key == ord('D'):
            mode = 'odometry'
        else:
            vL *= 0.75
            vR *= 0.75
        
        left_motor.setVelocity(vL)
        right_motor.setVelocity(vR)
        
        arm_joint_indices_inc = {
            ord('1'): 3,
            ord('2'): 4,
            ord('3'): 5,
            ord('4'): 6,
            ord('5'): 7,
            ord('6'): 8,
            ord('7'): 9
        }
        
        arm_joint_indices_dec = {
            ord('Q'): 3,
            ord('W'): 4,
            ord('E'): 5,
            ord('R'): 6,
            ord('T'): 7,
            ord('Y'): 8,
            ord('U'): 9
        }
        
        print("Key code:", key, "| Char:", chr(key) if 32 <= key <= 126 else '')
        
        if 'arm_joint_positions' not in globals():
            arm_joint_positions = {i: robot_parts[i].getTargetPosition() for i in range(3, 10)}
        
        if key in arm_joint_indices_inc:
            j_idx = arm_joint_indices_inc[key]
            arm_joint_positions[j_idx] += 0.025
            robot_parts[j_idx].setPosition(arm_joint_positions[j_idx])
            print("Incrementing Test")
        
        if key in arm_joint_indices_dec:
            j_idx = arm_joint_indices_dec[key]
            arm_joint_positions[j_idx] -= 0.025
            robot_parts[j_idx].setPosition(arm_joint_positions[j_idx])
            print("Decrementing Test")
            
        if 'torso_position' not in globals():
            torso_position = robot_parts[2].getTargetPosition()
            
        if key == ord('Z'):
            torso_position = min(torso_position + 0.02, 0.35)  
            robot_parts[2].setPosition(torso_position)
            print(f"Raising torso to {torso_position:.2f}")
    
        elif key == ord('X'):
            torso_position = max(torso_position - 0.02, 0.0)  
            robot_parts[2].setPosition(torso_position)
            print(f"Lowering torso to {torso_position:.2f}")
            
        if key == ord('O'):
            print("Opening Gripper")
            robot_parts[12].setPosition(0.045)
            robot_parts[13].setPosition(0.045)
        elif key == ord('C'):
            print("Closing Gripper")
            robot_parts[12].setPosition(0)
            robot_parts[13].setPosition(0)
    
    elif mode == 'odometry':
        size = len(current_path_grid)
        if index < size:
                if current_path_grid[index] is None or current_waypoint_index >= len(current_path_grid[index]):
                    print("Error: Navigation state entered with no valid path or index out of bounds.")
                    # This might happen if the goal was reached exactly on the last step of previous leg
                    # Treat as goal reached
                    goal_bool = True
                    robot_state = 'GOAL_REACHED'
    
                else:
                    # Get current target waypoint (grid and world coordinates)
                    target_grid = current_path_grid[index][current_waypoint_index]
                    target_world_wp = grid_to_world(target_grid[0], target_grid[1])
                    # print("target world waypoint o and 1", target_world_wp[0], target_world_wp[1])
    
                    # Calculate errors to the target waypoint
                    pos_err, bearing_err = calculate_errors(pose_x, pose_y, pose_theta, target_world_wp[0], target_world_wp[1])
    
                    # --- Simple P Controller Logic ---
                    if abs(bearing_err) > BEARING_THRESHOLD_FOR_FORWARD:
                        # Turn in place
                        vL = -KP_ANGLE * bearing_err
                        vR = KP_ANGLE * bearing_err
                        # Clamp speeds
                        vL = max(-MAX_SPEED, min(MAX_SPEED, vL))
                        vR = max(-MAX_SPEED, min(MAX_SPEED, vR))
                        # print(f"Turning: BearErr={math.degrees(bearing_err):.1f}deg | vL={vL:.2f}, vR={vR:.2f}")
                    else:
                        # Go forward and turn slightly
                        forward_component = FORWARD_SPEED_WHEN_ALIGNED # Constant speed when aligned
                        turn_component = KP_ANGLE * bearing_err # Small adjustments
    
                        vL = forward_component - turn_component
                        vR = forward_component + turn_component
                        # Clamp speeds
                        vL = max(-MAX_SPEED, min(MAX_SPEED, vL))
                        vR = max(-MAX_SPEED, min(MAX_SPEED, vR))
                        # print(f"Moving: PosErr={pos_err:.2f}m, BearErr={math.degrees(bearing_err):.1f}deg | vL={vL:.2f}, vR={vR:.2f}")
    
    
                    # Check if waypoint is reached
                    if pos_err < WAYPOINT_REACHED_THRESHOLD:
                        print(f"Waypoint {current_waypoint_index} / {len(current_path_grid[index])-1} reached (Grid: {target_grid}).")
                        current_waypoint_index += 2
                        print(index)
    
                        # Check if this was the last waypoint in the current path
                        if current_waypoint_index >= len(current_path_grid[index]):
                            print("GOAL REACHED")
                            robot_state = 'GOAL_REACHED' # Mark the major goal as reached
                            goal_bool = True
    
                    robot_parts[MOTOR_LEFT].setVelocity(vL)
                    robot_parts[MOTOR_RIGHT].setVelocity(vR)
    
                if robot_state == 'GOAL_REACHED':
                    print(f"--- Initial Goal {index} Reached! ---")
                    
                    robot_pickup(index, pose_x, pose_y, pose_theta)
                    
                    index = index + 1
                    current_waypoint_index = 0
                  
                    vL, vR = 0, 0 # Stop the robot momentarily
                    left_motor.setVelocity(vL)
                    right_motor.setVelocity(vR)
                    robot.step(timestep*5) # Pause briefly
                    
                    mode = 'manual'
                    
                    robot_state = 'not_goal'
    
                    # current_goal_index += 1 # Move to the next goal index
    
                    # Check if all goals have been reached
                    if index >= len(current_path_grid):
                        print(">>> All goals reached! Mission accomplished. <<<")
                            
    
    