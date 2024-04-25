# Walking-Darwin-OP2-Gazebo

# Darwin OP2 Simulation Packages (ROS 2)

This repository contains ROS 2 packages for simulating the Robotis Darwin OP2 robot using the logic originally created by [Yusuke-Yamasaki-555](https://github.com/open-rdc/ROS2_Walking_Pattern_Generator.git), but adapted for Gazebo. Please refer to their repository first for the original implementation.

In addition to a MoveIt2 control and rutine generator.

## Overview

The Robotis Darwin OP2 is a humanoid robot known for its versatility and agility. This simulation package aims to provide a realistic simulation environment for testing and developing algorithms for the Darwin OP2.

## Dependencies

ros2_control: [Getting Started](https://control.ros.org/humble/doc/getting_started/getting_started.html)
MoveIt2: [Binary Install ROS Humble](https://moveit.ros.org/install-moveit2/binary/)

## Installation

To use these simulation packages, follow these steps:

1. Clone this repository to your ROS 2 workspace:

   ```bash
   git clone https://github.com/Fernando0211/https://github.com/Fernando0211/Walking-Darwin-OP2-Gazebo.git

2. Build your ROS 2 workspace:

   ```bash
   colcon build

3. Source your workspace:

   ```bash
   source install/setup.bash

## Usage

After installation, you can launch the simulation environment using:

   ```bash
ros2 launch robot_bringup robot_bringup_gazebo.launch.py

