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

- To run the simulation run the command:  
  `ros2 launch turtlebot3_sim simulation.launch.py`
