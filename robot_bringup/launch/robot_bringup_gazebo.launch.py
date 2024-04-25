import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    robot_description_launch_file = os.path.join(get_package_share_directory('robot_description'), 'models/robotis_op2/launch/robotis_op2_gazebo.launch.py')
    robot_control_launch_file = os.path.join(get_package_share_directory('robot_control'), 'launch/robot_control.launch.py')
    robot_bringup_launch_file = os.path.join(get_package_share_directory('robot_bringup'), 'launch/robot_bringup_launch.py')

    robot_handler_cmd = ExecuteProcess(
        cmd=["ros2", "run", "robot_gazebo", "robot_handler"],
        output="screen"
    )

    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch_file)
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_control_launch_file)
    ))

    ld.add_action(robot_handler_cmd)

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_bringup_launch_file)
    ))

    return ld
