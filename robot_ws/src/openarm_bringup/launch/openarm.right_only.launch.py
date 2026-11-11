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

"""Bring-up for a single (right) arm built on the bimanual skeleton.

The robot description is generated with bimanual:=true / left_arm:=false, so the
model keeps the ``world`` link and the body and only the right arm
(``openarm_right_*``) plus its ``openarm_right_hardware_interface`` component is
instantiated. Only the v1.0 description files implement the left_arm/right_arm
switches.
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Only v1.0 supports left_arm:=false, so restrict arm_type here.
V1_ARM_TYPES = {
    "v1.0", "v10", "v1_0", "openarm_v1.0", "openarm_v10", "openarm_v1_0",
}


def resolve_arm_config(arm_type_str: str) -> tuple[str, str]:
    """Resolve the description folder/file for a v1.0 arm_type."""
    if arm_type_str not in V1_ARM_TYPES:
        raise ValueError(
            f"Invalid arm_type: '{arm_type_str}'. openarm.right_only.launch.py "
            f"only supports v1.0 variants ({sorted(V1_ARM_TYPES)}); the "
            f"left_arm/right_arm switches are not implemented for v2.0 yet."
        )
    return "openarm_v1.0", "openarm_v10.urdf.xacro"


def generate_robot_description(context: LaunchContext, description_package,
                               arm_type, use_fake_hardware, right_can_interface,
                               can_fd):
    """Generate the right-arm robot description with xacro."""
    description_package_str = context.perform_substitution(description_package)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    can_fd_str = context.perform_substitution(can_fd)

    folder_name, file_name = resolve_arm_config(arm_type_str)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "assets", "robot", folder_name, "urdf", file_name
    )

    return xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "left_arm": "false",   # <-- single arm: body + right arm only
            "right_arm": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "right_can_interface": right_can_interface_str,
            "can_fd": can_fd_str,
        }
    ).toprettyxml(indent="  ")



def robot_nodes_spawner(context: LaunchContext, description_package, arm_type,
                        use_fake_hardware, controllers_file, right_can_interface,
                        can_fd, use_rviz, rviz_config):
    """Start robot_state_publisher, ros2_control_node and (optionally) RViz."""
    robot_description = generate_robot_description(
        context, description_package, arm_type, use_fake_hardware,
        right_can_interface, can_fd,
    )

    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    nodes = [robot_state_pub_node, control_node]

    if context.perform_substitution(use_rviz).lower() == "true":
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", context.perform_substitution(rviz_config)],
        ))

    return nodes


def joint_state_broadcaster_spawner(context: LaunchContext, controllers_file):
    """Spawn joint_state_broadcaster (first, so /joint_states exists early)."""
    del context, controllers_file
    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
            ],
            output="screen",
        ),
    ]


def arm_controller_spawner(context: LaunchContext, robot_controller):
    """Spawn the right arm and right gripper controllers."""
    context_value = context.perform_substitution(robot_controller)
    controller = "right_joint_trajectory_controller"
    if context_value == "forward_position_controller":
        controller = "right_forward_position_controller"
    elif context_value != "joint_trajectory_controller":
        raise ValueError(f"Unknown robot_controller: {context_value}")
    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_gripper_controller",
                "--controller-manager", "/controller_manager",
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    """Generate the launch description for the right-arm-only setup."""
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="v10",
            description="Arm type. Only v1.0 variants are supported here.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware (mock_components) instead of the real arm.",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            description="Right arm controller: joint_trajectory_controller or "
                        "forward_position_controller.",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can0",
            description="CAN interface of the right arm.",
        ),
        DeclareLaunchArgument(
            "can_fd",
            default_value="true",
            description="Use CAN-FD. Set to false for CAN 2.0.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_bimanual_controllers.yaml",
            description="Controllers file (the right_* controllers are reused).",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with the bimanual RViz configuration.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    right_can_interface = LaunchConfiguration("right_can_interface")
    can_fd = LaunchConfiguration("can_fd")
    controllers_file = LaunchConfiguration("controllers_file")
    use_rviz = LaunchConfiguration("use_rviz")

    controllers_file_path = PathJoinSubstitution(
        [FindPackageShare("openarm_bringup"), "config", "controllers",
         controllers_file]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "bimanual.rviz"]
    )

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, arm_type, use_fake_hardware,
              controllers_file_path, right_can_interface, can_fd,
              use_rviz, rviz_config]
    )

    joint_state_broadcaster_spawner_func = OpaqueFunction(
        function=joint_state_broadcaster_spawner,
        args=[controllers_file_path]
    )

    arm_controller_spawner_func = OpaqueFunction(
        function=arm_controller_spawner,
        args=[robot_controller]
    )

    return LaunchDescription(
        declared_arguments + [
            robot_nodes_spawner_func,
            # Serialise the spawners: the controller_manager locks its resources
            # while (un)loading a controller, so concurrent spawners time out.
            TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner_func]),
            TimerAction(period=4.0, actions=[arm_controller_spawner_func]),
        ]
    )
