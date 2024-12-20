# West Point Turtlebot3 Robot Description

- This package was created to provide turtlebot3 capabilites with ROS2 Jazzy and Gazebo Harmonic

## Code additions and modifications

- The code in this repo is combination of the below sources:
  - <https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation.git>
  - <https://github.com/ROBOTIS-GIT/turtlebot3.git>
- Only the code mentioned below has been updated and tested
- The following directories and there content have been added:
  - turtlebot3_description/configs
  - turtlebot3_description/hooks
  - turtlebot3_description/launch
- Also the following files have been added or modified:
  - turtlebot3_description/urdf/turtlebot3_waffle_pi.urdf.xacro
  - turtlebot3_description/rviz/config.rviz
  - turtlebot3_description/CMakeLists.txt
- The following package was created by copying and modifying the tb4 equivalent in the nav2_minimal_turtlebot_simulation package
  - turtlebot3_sim

## Running the simulation

- To run just the simulation run the command:  
  `ros2 launch turtlebot3_sim simulation.launch.py`  
- To run SLAM run these two commands:  
  `ros2 launch turtlebot3_sim simulation.launch.py`  
  `ros2 launch turtlebot3_cartographer cartographer.launch.py`  
  - Cartographer launch has been modified to also run map server. To save a map use the command:
    `ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"`
- To run Nav2 in the Turtlbot3 World run these two commands:  
  `ros2 launch turtlebot3_sim simulation.launch.py`  
  `ros2 launch turtlebot3_navigation2 navigation2.launch.py`
  - Then set the robot initial pose in rviz. In the simulation the robot is facing up and aligned with the left most obstacles. If no changes were made to RVIZ, the robot's initial pose should be facing to the right and aligned with the top row of obstacles.
  - Navigate to waypoints using RVIZ
  