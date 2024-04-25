import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    op2_description_share_dir = get_package_share_directory("op2_description")
    op2_launch_path = os.path.join(
        op2_description_share_dir,
        "launch",
        "gazebo.launch.py"
    )

    op2_control_share_dir = get_package_share_directory("op2_control")
    op2_control_path = os.path.join(
        op2_control_share_dir,
        "launch",
        "op2_control.launch.py"
    )

    op2_moveit_share_dir = get_package_share_directory("op2_moveit")
    op2_moveit_path = os.path.join(
        op2_moveit_share_dir,
        "launch",
        "op2_moveit.launch.py"
    )

    ld.add_action(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(op2_launch_path),
        )
    )

    ld.add_action(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(op2_control_path),
        )
    )

    ld.add_action(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(op2_moveit_path),
        )
    )

    return ld
