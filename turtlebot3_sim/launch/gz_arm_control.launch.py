# Copyright (C) 2023 Open Source Robotics Foundation
# Copyright (C) 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is modified from the all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    sim_dir = get_package_share_directory('open_manipulator_x_bringup')

    # Launch configuration variables specific to simulation
    arm_joint_controller = LaunchConfiguration("arm_joint_controller")
    gripper_joint_controller = LaunchConfiguration("gripper_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")

    declare_activate_joint_controller_cmd = DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    
    declare_arm_joint_controller_sdf_cmd = DeclareLaunchArgument(
            "arm_joint_controller",
            default_value="arm_controller",
            description="Robot controller to start.",
        )
    
    declare_gripper_joint_controller_sdf_cmd = DeclareLaunchArgument(
            "gripper_joint_controller",
            default_value="gripper_controller",
            description="Robot controller to start.",
        )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # There may be other controllers of the joints, but this is the initially-started one
    arm_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[arm_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(activate_joint_controller),
    )
    arm_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[arm_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(activate_joint_controller),
    )

    gripper_controller_spawner_started = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[gripper_joint_controller, '--param-file', os.path.join(sim_dir, 'config', 'gz_gripper_controller_manager.yaml'), "-c", "/controller_manager"],
        condition=IfCondition(activate_joint_controller),
    )

    gripper_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[gripper_joint_controller, '--param-file', os.path.join(sim_dir, 'config', 'gz_gripper_controller_manager.yaml'), "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(activate_joint_controller),
    )


    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_activate_joint_controller_cmd)
    ld.add_action(declare_arm_joint_controller_sdf_cmd)

    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(arm_joint_controller_spawner_started)
    ld.add_action(arm_joint_controller_spawner_stopped)
    #ld.add_action(spawn_gripper_controller)


    return ld
