# Copyright 2025 Enactic, Inc.
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

"""
Gazebo Harmonic (gz sim) + RViz co-simulation for the OpenArm v2.0 bimanual arm.

Startup order is event driven (no hard-coded sleeps):
    gz sim -> spawn model from /robot_description -> joint_state_broadcaster
           -> left/right arm controllers -> left/right gripper controllers

The controllers are the ones described in config/gazebo/controllers.yaml, which
is loaded by the gz_ros2_control plugin injected by the URDF when the xacro is
expanded with sim:=gz.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARM_CONTROLLERS = ("left_arm_controller", "right_arm_controller")
GRIPPER_CONTROLLERS = ("left_gripper_controller", "right_gripper_controller")

# Accepted arm_type values, kept in sync with display_openarm.launch.py
VALID_ARM_TYPES = {
    "v1.0", "v10", "v1_0", "openarm_v1.0", "openarm_v10", "openarm_v1_0",
    "v2.0", "v20", "v2_0", "openarm_v2.0", "openarm_v20", "openarm_v2_0",
}


def resolve_arm_config(arm_type_str: str) -> tuple[str, str]:
    """Map an arm_type string to (asset folder, entry xacro file)."""
    if arm_type_str not in VALID_ARM_TYPES:
        raise ValueError(
            f"Invalid arm_type: '{arm_type_str}'. "
            f"Accepted values: {sorted(VALID_ARM_TYPES)}."
        )
    if any(x in arm_type_str for x in ("1.0", "10", "1_0")):
        return "openarm_v1.0", "openarm_v10.urdf.xacro"
    return "openarm_v2.0", "openarm_v20.urdf.xacro"


def _spawner(controller_name: str) -> Node:
    """Controller spawner that tolerates a slow-to-appear controller_manager."""
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("openarm_description")

    arm_type = context.perform_substitution(LaunchConfiguration("arm_type"))
    robot_preset = context.perform_substitution(LaunchConfiguration("robot_preset"))
    world = context.perform_substitution(LaunchConfiguration("world"))
    gui = context.perform_substitution(LaunchConfiguration("gui")).lower() in ("true", "1")

    folder_name, file_name = resolve_arm_config(arm_type)
    xacro_path = os.path.join(pkg_share, "assets", "robot", folder_name, "urdf", file_name)

    # sim:=gz swaps the mock/CAN hardware for gz_ros2_control and injects the
    # GazeboSimROS2ControlPlugin block at the end of the URDF.
    if folder_name == "openarm_v1.0":
        # v1.0 has no robot presets, and ros2_control is off by default there.
        mappings = {
            "arm_type": "v10",
            "body_type": "v10",
            "bimanual": "true",
            "ros2_control": "true",
            "use_fake_hardware": "false",
            "fake_sensor_commands": "false",
            "sim": "gz",
        }
    else:
        mappings = {
            "robot_preset": robot_preset,
            "sim": "gz",
        }

    robot_description = xacro.process_file(
        xacro_path,
        mappings=mappings,
    ).toprettyxml(indent="  ")

    world_path = world if os.path.isabs(world) else os.path.join(pkg_share, "worlds", world)
    # -r: run immediately, -s: server only (no GUI), -v 4: verbose
    gz_args = f"-r -v 4 {world_path}" if gui else f"-s -r -v 4 {world_path}"

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Without this bridge every use_sim_time node stays frozen at t=0.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "true",
        }.items(),
    )

    # -topic robot_description: read the URDF straight from the RSP latched topic
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_openarm",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "openarm",
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
    )

    joint_state_spawner = _spawner("joint_state_broadcaster")
    left_arm_spawner = _spawner(ARM_CONTROLLERS[0])
    right_arm_spawner = _spawner(ARM_CONTROLLERS[1])

    event_handlers = [
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_spawner])
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[left_arm_spawner, right_arm_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=left_arm_spawner,
                on_exit=[_spawner(name) for name in GRIPPER_CONTROLLERS],
            )
        ),
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", os.path.join(pkg_share, "rviz", "bimanual.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return [
        robot_state_publisher,
        clock_bridge,
        gz_sim,
        spawn_robot,
        *event_handlers,
        rviz_node,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("openarm_description")

    # package://openarm_description/... only resolves in Gazebo when the parent
    # directory of share/openarm_description is on GZ_SIM_RESOURCE_PATH.
    gz_resource_path = os.path.dirname(pkg_share)
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_resource_path:
        gz_resource_path = gz_resource_path + os.pathsep + existing_resource_path

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
        DeclareLaunchArgument(
            "arm_type",
            default_value="v10",
            description="Arm type: v10/v1.0 (default, our hardware) or v20/v2.0.",
        ),
        DeclareLaunchArgument(
            "robot_preset",
            default_value="default_bimanual",
            description="Robot preset for v2.0 only (e.g. default_bimanual, right_arm, left_arm).",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="openarm_empty.sdf",
            description="World file name inside the worlds/ folder, or an absolute path.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start the Gazebo GUI. Set false for headless simulations.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2 alongside Gazebo.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
