# ROS Project: Control Layer Between Unity and Doosan Robotic Arm

## Project Overview

This project aims to create a control layer between a Unity VR project and the ROS drivers of a Doosan robotic arm. It enables seamless communication between Unity and ROS, allowing real-time control and manipulation of the robotic arm using VR inputs.

## Features
- Control layer transforming messages for communication between Unity and ROS.
- Support for Doosan robotic arm with ROS drivers.
- Integration with Realsense cameras and Robotiq gripper.
- Real-time control using Unity VR and ROS.

## Installation Instructions

This project cannot function independently, as some launch files require other projects.

### Prerequisites

- **Operating System**: This project was developed using ROS Noetic on Ubuntu 20.04.
- **Doosan robot**: The scripts in this project work with the ROS Doosan drivers available [here](https://github.com/ETS-J-Boutin/doosan-robot_RT), a modified version of the [original](https://github.com/BryanStuurman/doosan-robot) branch. For detailed setup instructions, refer to this [issue](https://github.com/doosan-robotics/doosan-robot/issues/99).
- **Realsense Camera**: The cameras used in this project are Realsense cameras. Their drivers can be found [here](https://github.com/rjwb1/realsense-ros).
- **Robotiq Gripper**: The gripper used is a Robotiq 2f 85. An updated fork compatible with ROS Noetic is available [here](https://github.com/alexandre-bernier/robotiq_85_gripper).
- **Unity-ROS Bridge**: This can be found at the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) and is used to connect Unity to ROS. For tutorials and guidance on setting up ROS in Unity, including the ROS-TCP-Endpoint, follow their [ROS_setup tutorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/0_ros_setup.md). ROS Noetic was used for this project. Note: you will need to change the first line of code of the file ROS-TCP-Endpoint/src/ros_tcp_endpoint/default_server_endpoint.py to
```bash
#!/usr/bin/env python3
```
This ensures that the Endpoint works with Python 3.

By adding these to your `src` folder, you should have, including this project, a folder for the following projects:
- Doosan robot
- Realsense
- Robotiq 85
- ROS TCP Endpoint
- VR Unity ROS Doosan (this project)

Make sure to run `catkin_make` after all these installations.

## Usage

These are the commands you might want to run.

### rosrun

- **go_home.py**: Makes the robot go to a home position using the velocity controller of the robot. This script should not be run at the same time as other messages are published to move the robot on the `/dsr_joint_velocity_controller/command` ROS topic.

### roslaunch

- **keyboard_gripper.launch**: Launches the Robotiq 2f 85 drivers and makes the gripper open or close when pressing the keyboard spacebar. Holding the spacebar will make the gripper fluctuate between the two states.
- **moveit_servo_doosan.launch**: Launches the moveit_servo real-time arm servoing drivers with the Doosan config file. This enables manipulation of the robot by using a Cartesian target for where the end-effector of the robot needs to go.
- **vr_realtime.launch**: Starts the drivers of the Realsense camera and the Robotiq gripper. It also uses the scripts that act as a control layer between Unity and ROS. Do not use it at the same time as the keyboard_gripper.launch.

### Example

Launch the various ROS components needed for real-time manipulation using the following commands in multiple command windows:

  ```bash
  roslaunch dsr_ros_control doosan_interface_moveit.launch # Launch the ROS driver for real-time control.
  roslaunch unity_ros_doosan moveit_servo_doosan_cpp.launch 
  roslaunch unity_ros_doosan vr_realtime.launch # Launches camera, gripper, and conversion scripts. Permissions for the gripper may need adjustment (`sudo chmod 777 /dev/ttyUSB0`).
  roslaunch ros_tcp_endpoint endpoint.launch # Launch the Unity-ROS bridge
  rosrun rqt_controller_manager rqt_controller_manager # Switch Doosan controller to velocity control.
