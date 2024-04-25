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
            os.path.join(get_package_share_directory("robot_description"), "models", "robotis_op2", "urdf", "robotis_op2.urdf.xacro")
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

    controllers = [
        "robot_controller"
        # "HeadPan_controller",
        # "HeadTilt_controller",
        # "RShoulderPitch_controller",
        # "RShoulderRoll_controller",
        # "r_el_controller",
        # "LShoulderPitch_controller",
        # "LShoulderRoll_controller",
        # "l_el_controller",

        # "RHipYaw_controller",
        # "RHipRoll_controller",
        # "RHipPitch_controller",
        # "RKnee_controller",
        # "RAnklePitch_controller",
        # "r_ank_roll_controller",

        # "l_hip_yaw_controller",
        # "LHipRoll_controller",
        # "LHipPitch_controller",
        # "LKnee_controller",
        # "LAnklePitch_controller",
        # "LAnkleRoll_controller"
    ]


    controller_nodes = []


    for controller in controllers:
        node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"]
        )
        controller_nodes.append(node)


    return LaunchDescription([
        robot_state_publisher,
        #joint_state_broadcaster_spawner,
        *controller_nodes
    ])
