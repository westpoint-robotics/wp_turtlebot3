# WP Turtlebot3 with Open Manipulator in Gaebo Harmonic

- Start the simulation with the Tbot with Arm  
  `ros2 launch turtlebot3_sim arm_sim.launch.py`

- You can drive the robot with cmd_vel

- You can see the arm moving with the example python code. NOTE: It takes about 12 seconds for the arm to start moving.  
  `ros2 launch open_manipulator_x_bringup test_joint_trajectory_controller.launch.py`

- You can control the arm with MoveIt RVIZ Plugin:  
  `ros2 launch open_manipulator_x_moveit_config moveit_gz.launch.py`



# Nav2 Minimal Turtlebot4 Simulation

This is a minimum simulation for the Turtlebot4 for use with Nav2. This is setup to strip out external dependencies and complexities of the iRobot Create3 & TB4 ignition simulation panels to the bare minimum needed for simulating the robot for Nav2's bringup which hopefully will not require changes from distribution-to-distribution.

Includes the Gazebo bridge configuration and launch files to open the robot in a particular simulated world (provided)
