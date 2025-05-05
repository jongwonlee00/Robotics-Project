##FINAL ROBOTICS PROJECT##

****

Contributors:
Jongwon Lee, 
Anja Delzell, 
Jack Toenjes

****

⚠️ Note: Code must be run on our custom world file, as changes were made to the starting position of the TIAGo robot.

🚀 Project Summary
****
Our robot implements a fully integrated pipeline combining mapping, path planning, navigation, computer vision, and manipulation. All subsystems are interdependent to enable the robot to autonomously detect, navigate to, and pick up yellow and green cubes in a grocery store environment.

✅ Core Features
****

Autonomous Mapping (Lead: Anja)
The robot uses LiDAR to autonomously generate a map. Simultaneously, it detects yellow and green blobs and stores the robot's GPS position as an initial waypoint upon detection.

Odometry + Navigation (Lead: Jack)
The robot navigates to waypoints using position and bearing errors fed into a proportional controller. When thresholds are met, manual manipulation is triggered.

Computer Vision (Lead: Jon)
The robot captures RGB frames, converts them to HSV color space, uses OpenCV to extract contours, filters blobs based on size/aspect ratio, and stores centroid GPS positions if unique.

RRT Path Planning (Anja & Jack)
A modified RRT algorithm plans paths on a grid-based convolved map. Paths are stored as numpy arrays and converted back to world coordinates for navigation.

Manual Arm Manipulation (Lead: Jon)
The robot supports keyboard-controlled joint manipulation to fine-tune object grasping.

📌 Keyboard Commands
Key	Action
1–7	Raise arm joints
q–u	Lower arm joints
Z / X	Raise / Lower torso
O / C	Open / Close gripper
D	Done with pick/place (return to odometry)

🗺️ Mapping
Lead: Anja Delzell
Target Date: 4/25 | Completed: 4/29
✅ Implemented LiDAR navigation, convolved map for RRT, autonomous isle-following logic.
📁 Output: auto_map.npy, goal_waypoints.npy

📍 Localization & Odometry
Lead: Jack Toenjes
Target Date: 4/25 | Completed: 4/30
✅ Calculates heading and position error, indexes through waypoints, uses a proportional controller.
📁 Output: current_path_grid.npy

🎯 Navigation (RRT)
Leads: Jack Toenjes, Anja Delzell
Target Date: 4/18 | Completed: 4/30
✅ Pathfinding on grid with obstacle checking.
❌ A* not implemented due to complexity.

👁️ Computer Vision
Lead: Jongwon Lee
Target Date: 4/30 | Completed: 4/30
✅ Converts camera data to HSV, applies threshold masking, filters by area/aspect ratio, detects blobs.
📁 Output: real-time detection and goal_waypoints.npy

🤖 Manipulation
Leads: Jongwon Lee, Jack Toenjes
Target Date: 4/30 | Completed: 5/2
✅ Manual joint control via keyboard
❌ No IK / automatic arm planning
📁 Controlled via custom manual mode after waypoint arrival

🔗 System Integration
Every module is interconnected:

Computer Vision runs during Autonomous Mapping.

RRT plans paths for Odometry Navigation.

Odometry leads to Manual Manipulation zones.

Keyboard Joint Control is used once aligned for pickup.

