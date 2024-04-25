from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    robot_description = ParameterValue(
        Command(
        [
            "xacro ",
            os.path.join(get_package_share_directory("op2_description"), "urdf", "op2.urdf.xacro")
        ]
        ),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            ]
    )

    head_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "head_controller",
            "--controller-manager",
            "/controller_manager",
            ]
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager",
            "/controller_manager",
            ]
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager",
            "/controller_manager",
            ]
    )

    r_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "r_leg_controller",
            "--controller-manager",
            "/controller_manager",
            ]
    )

    l_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "l_leg_controller",
            "--controller-manager",
            "/controller_manager",
            ]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        head_controller_spawner,
        right_arm_controller_spawner,
        left_arm_controller_spawner,
        r_leg_controller_spawner,
        l_leg_controller_spawner
    ])
