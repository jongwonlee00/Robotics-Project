Robotics Final Project

Jongwon Lee
Anja Delzell
Jack Toenjes

Project Summary

*Code must be run on our world as changes are made to the starting position of the tiago. 

Our robot implements the following features:

Autonomous Mapping : Anja
The robot moves autonomously capturing the whole map using its lidar sensor. Color blob detection is called simultaneously, which stores the position of the robot at the time it detects yellow or green blobs as an initial waypoint. 

Odometry : Jack
Once the robot reaches an initial waypoint near a cube, it repeatedly runs calculations on its bearing and position errors. These error values are fed into a proportional controller. When error values are within the threshold, the program runs manual mode for pick and place. 

Computer Vision : Jon
Uses Webot camera to extract image data, convert to HSV color space for higher accuracy, then use opencv color threshold matching to isolate blobs. Contours are found on the binary mask, filtered by area and aspect ratio, centroids of the blobs are extracted using a similar method from homework 3. In autonomous mapping, a color blob function is called every timestep, and when detected, it saves robots' current pose using gps. 

Navigation using RRT : Anja, Jack
Homework 3 was used as a base code for the RRT algorithm. Jack built a base code to work on our map, then Anja added the obstacle avoidance logic. Algorithm takes in grid points instead of the world, making it less complex. Code converts back to world coordinates for odometry.

Manual Manipulation : Jon
Keyboard input was used  to increment or decrement the robot's arm joint and torso by 0.025 radians. 
Manual key command follows:
 - 1-7 = Raise arm joints
 - q-u = Lower arm joints
 - Z/X = Raise/Lower torso
 - D = Done with pick and place. (switches back to odometry)
 - O/C = Open/Close gripper

Connecting the dots 
Every code in this project is interdependent. Computer vision runs during autonomous mode. RRT works during the planning algorithm. Odometry uses RRT for accurate navigation. Manipulation is in odometry to be able to pick and place while moving from cube to cube. 
















Mapping
Lead: Anja Delzell
Target Completion Date: 4/25
Actual Completion Date: 4/29
Figure 1-7
Tier 
Completion (✅ /❌)
To-Do
Delivery
Date
2
✅
Keyboard control for navigation
4/24
2
✅
Adjust Lidar Heights
4/24
2
✅
Display map
4/24
2
✅
Reduce Lidar noise
4/24
2
✅
Robot navigation around the map
4/27
2
✅
Complex navigation logic
4/27
2
✅
Convolve map for RRT planning
4/27
2
✅
RRT for all goal positions and save path as 2D 
numpy array.
4/29

Autonomous planning is the mode we use to actually run our RRT algorithm and get the waypoints needed to move around the map and pick up blocks. It requires all the same helper functions as Manual planning and also calls RETin a for loop of the size of the number of goal points we have so we can map the waypoints for each goal point. 
Some of the code has been integrated from Jacks original RRT algorithm, including the waiting for a gps signal, reconstruct path and RRT call. (There was some difficulty with integrating these together and making sure all values were coordinate pairs.) 
Use waypoints array like the one from Lab 5 to make the robot move on its own once started. This required first making an implementation that moved autonomously. The implementation I used begins in the corner and assumes there are isles on the map as we are in a grocery store. It then moves left and then right to follow down the isles until it can no longer go down an isle in which it stops. I have moved the lidar sensor up to 0.97 m to include shelves and the table in my readings. I then convolve the map for the autonomous sensor in autonomous planning mode. 


Localization
Lead: Jack Toenjes 
Target Completion Date: 4/25
Actual Completion Date: 4/30
Figure 8-10
Tier 
Completion (✅ /❌)
To-Do
Delivery
Date
2
✅ 
 Calculate Errors (heading, bearing, position)
4/18
2
✅ 
Waypoint following logic
4/25
2
✅
RRT implementation
4/29
2
✅ 
Indexing through waypoints
4/30
2
✅ 
Proportional Controller
4/30

	The odometry code we made has now been integrated into our final project and works after RRT gets implemented. It is included in an odometry mode. Some struggles with getting these to work with rrt included updating the grid to world function so it accounted for potential rounding errors. 
The updated code also contains an index and size of the waypoints list so that we can index through the number of waypoints. 







Computer Vision 
Lead: Jongwon Lee
Target Completion Date: 4/30
Actual Completed Date: 4/30
Figure 11
Tier 
Completion (✅ /❌)
To-Do
Expected Delivery
2
✅ 
Initialize Webot Camera 
4/16
2
✅ 
Capture RGB frame from Webot Camera 
4/16
2
✅
RGB converted to HSV color space
4/24
2
✅
Mask defined using opencv
4/24
2
✅
Extracted contours
4/24
2
✅
Filtering by area and aspect ratio
4/24
2
✅
Print centroid location and custom bounding box
4/28
2
✅ 
Green blob logic implementation
4/30
2
✅ 
Autonomous mapping implementation
4/30

The robot’s camera captures an image, which is retrieved as raw RGB pixel data from Webots and manually converted to a NumPy array in BGR format (as OpenCV expects BGR, not RGB). The BGR image is then converted to HSV (Hue, Saturation, Value) color space for higher accuracy (BGR threshold wasn’t working well). A binary mask is created using cv2.inRange() that filters out everything except pixels within the color range. Using cv2.findContours(), external contours are identified in the binary mask. Each contour's area is checked to filter out noise. The centroid of the blob is computed using image moments (cv2.moments()). A bounding box is drawn, and the aspect ratio of each blob is checked (should be around 1:1 for cubes). The GPS coordinates of the robot at the time of detection are recorded and stored if the centroid corresponds to a new, unique cube position (checked via a spatial threshold in a function called is_new_waypoint()). The mask and the frame are displayed using cv2.imshow(), showing detected rectangles and labeled centroids dot for real-time debugging and validation.
	Some difficulties encountered was that the 2D centroid (cx, cy) in the camera frame made it  hard to determine the real-world 3D position of the cube, which we didn’t need because we ended up doing keyboard manipulation. 
Navigation
Lead: Jack Toenjes/Anja Delzell
Target Completion Date: 4/18
Actual Completion Date: 4/30 
Tier 
Completion (✅ /❌)
To-Do
Delivery
Date
2
✅
 RRT algorithm
4/27
2
✅ 
Conversion to grid coordinates for obstacle check
4/27
2
✅ 
RRT parameters for less noise
4/27
2
✅ 
Obstacle checking functions using convolved map
4/30
2
✅
RRT store path in 2D numpy array and save file
4/30
2
✅
Reconstruct path is used to extract full waypoints for each goal positions
4/30

Jack and Anja have updated the already completed RRT algorithm to include the obstacles. The RRT algorithm was also updated so that its path is found in grid coordinates instead of world coordinates. This allows for easier obstacle detection because we detect at each grid point if there is an obstacle. This algorithm is implemented by functions RRT, reconstruct path, edge is valid, state is valid etc. These have all been updated to be in grid coordinates. 
The following A* was not implemented due to its complexity: We will implement the A* algorithm from lab 5 and use this in our project We will choose the order of our points ahead of time: Potentially we can go from aisle to aisle trying to pick up each object in the row of the store. A* will be used for each individual object. Every time we correctly get an object into our cart we can run the algorithm again





Manipulation
Lead: Jongwon Lee / Jack Toenjes
Target Completion Date: 4/30
Actual Completed Date: 5/2
Figure 12
Tier 
Completion (✅ /❌)
To-Do
Expected Delivery
1
✅
Implemented robot pick up pose navigation logic
5/2
1
✅
Hardcoded robot manipulation in joint space using keyboard inputs
5/2
2
❌
Transform the detected pose into the robot’s base frame
N/A
2
❌
Solve inverse kinematics (IK) to compute joint angles for reaching the target pose
N/A
2
❌ 
Move the arm to pick/place using the IK solution
N/A
2
❌ 
Define and hardcode a series of waypoints for navigation
N/A
2
❌ 
Implement navigation logic 
N/A


	Jon wrote a robot pick up function that is used for manipulation. The initial goal waypoint implemented during autonomous mode isn’t quite as close to the cube as we want. Therefore, we hardcoded a few more positions nearby cube to follow after the robot reached those initial goal waypoints. Once it reaches more sophisticated hard coded waypoints (final waypoints), it enters manual mode, which runs keyboard manipulation code. Arm joints and torso are adjusted accordingly. 

Keyboard Commands 

1-7 = Raise arm joints
 q-u = Lower arm joints
 Z/X = Raise/Lower torso
 D = Done with pick and place. (switches back to odometry from manual)
 O/C = Open/Close gripper



Original Project Plan

Mapping SLAM - (Live simulation) 
Live simulation shall trigger robots movement autonomously, also allowing manual input, showing correct responsiveness 

Localization SLAM/MCL (Live simulation + terminal output)  
Simulation updates robot’s pose accurately on terminal output, robot’s behavior reflects accordingly

Computer Vision through ML/DL (Live simulation + terminal output)
Simulation accurately detects objects thru bounding boxes, terminal output shows detection details (color, shape, etc) with confidence score, simulation should be in sync with its detection

Navigation using RRT w/ path smoothing (Terminal output)
Simulation generates a RRT graph as a output file, highlighting the optimal path stored in file, path should lead robot to a yellow cube while avoiding obstacles

Autonomous Manipulation (Live Simulation) 
Robot’s camera will display the robot's arm approaching the object, arm being able to grasp the item and place it in the basket using inverse kinematics, and avoiding obstacles with hardcoded waypoints (test : will Tiago collide with obstacles on occasion?)












Mid Project Updates

Anja: Final Project Check In (4/21): 

The main task I have been given is to figure out how to Autonomously Map our robot throughout the grid. Thus far I have set up the map for the new project and am displaying a rectangular display that shows where obstacles have been. I have put in many hours in getting the robot to work as I need it to and the grid size to be correct, as now the layout of the room is a rectangle instead of a square. I have also implemented some helper functions and modified some helper functions from lab five. Previously we did not have a Convolve function, a grey scale function, and a save map function. I have added these to improve the flow of the code and eliminate repetition in my code. I have also modified the world to grid and grid to world functions to hopefully work correctly on my display.

Struggles with this part include figuring out how to map this grid autonomously and better set up the map in real time. I want to make sure that when I save my Map it does not have a bunch of extra points due to noise. Further, autonomously mapping seems to be a very difficult task. I plan to have the robot first track the perimeter and then run through the center of the map following each obstacle until it has mapped fully around it. I am unsure of how to implement this quite yet but will continue to work on this. I will try to implement some sort of algorithm that looks at where I have already gone so I can look at where I still need to go. I have also been struggling with a lot of minor errors in my code. It seems as if every line of code I implement has some sort of error. 

One question I have is if for my robot to be autonomous it cannot have any previous knowledge of the map. I know in lab five we did a robot that had previous knowledge but I do not believe that was for mapping so I just want to be sure I understand what I am supposed to be doing. 
Ultimately I am making some good progress on the mapping of my world but have not quite figured out how to make it autonomous yet. This seems like a complex problem and I will put as much effort as I can into making it happen. 

Jon: Final Project Check In (4/21) : 
Currently, Color Blob Detection Tier 2 has been completed with one pending update. For finding yellow cubes, I wrote down my own color blob detection using raw RGB values from the camera. I've disabled built-in camera.recognition from Webot. We loop through every pixel using imageGetRed, Green, and Blue to check for yellow based on thresholds. Once we find a yellow pixel, we group it to blob and calculate the bounding box and centroid. We filter blobs based on size and shape to match yellow cube so that we don’t accidentally pick up random yellow objects like honey jars. I believe the current implementation is eligible to receive 18 points of computer vision grading rubric. I look forward to possibly implementing machine learning through CNN. That will come after meeting minimum requirements for manipulation. 
Jack: Final Project Check in 4/21: 
I am currently using hard-coded waypoints for my robot to follow, but that will change when RRT is implemented correctly. The RRT algorithm should give me a valid coordinate space where I can put my waypoints, so I won’t have to worry about finding waypoints clear of obstacles.
My vision is to use the steer and get_nearest_vertex functions from homework 2, but the RRT function will have to be modified quite a bit for this project. My starting point will be an element of the waypoints array (a coordinate), and my goal point will be the next element unless index>=the size of the waypoints array. I am working on getting obstacle coordinates that I can use for the grocery store environment. I may need to change k/delta_q, but they are fine for now. 


