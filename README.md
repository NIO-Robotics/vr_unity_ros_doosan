# ROS projet used as a control layer between a Unity project and a Doosan robotic arm.

This project consist of different scripts and launch files that act as a control layer between a Unity VR project and the ROS drivers of the robot. This layer transform messages so that both system can communicate with one another.

# Installation instruction

This project doesn't work in itself, as some launch file requires other project.

## Prerequisites

- **Operating System**: This project was developped using ROS Noetic on Ubuntu 20.04.
- **Doosan robot**: The scripts made in this project work with the ROS Doosan drivers that can be found [here](https://github.com/ETS-J-Boutin/doosan-robot_RT). Which is a modify version of the [original](https://github.com/BryanStuurman/doosan-robot) branch. For detailed setup instructions, refer to this [issue](https://github.com/doosan-robotics/doosan-robot/issues/99) which also explains the procedure.
- **Realsense Camera**: The camera used for this project are realsense cameras. Their drivers can be found [here](https://github.com/rjwb1/realsense-ros).
- **Robotiq Gripper**: The gripper used is a Robotiq 2f 85. An updated fork compatible with ROS Noetic is available [here](https://github.com/alexandre-bernier/robotiq_85_gripper).
* **Unity-ROS Bridge**: This can be found at the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) and is used to connect Unity to ROS. For tutorials and guidance on setting up ROS in Unity, including the ROS-TCP-Endpoint, follow their [ROS_setup tutorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/0_ros_setup.md). ROS Noetic was used for this project. **REQUIRED FOR ALL ROBOTS.** Note: you will need to change the first line of code of the file ROS-TCP-Endpoint/src/ros_tcp_endpoint/default_server_endpoint.py to
  ```bash
  #!/usr/bin/env python3
  ```
  This make sure that the Endpoint works with Pyhton 3.

By adding these to your src folder, you should have, including this project, a folder for the following project:
-Doosan robot
-Realsense
-Robotiq 85
-ROS TCP Endpoint
-VR Unity ROS Doosan (this project)

Make sure to do a catkin_make after all these installation.

# Usage

These are the command that you might want to run.

## rosrun

-**go_home.py**: Makes the robot go to a home position using the velocity controller of the robot. This script should no be run at the same time as other messages are published to moves the robot on the /dsr_joint_velocity_controller/command ROS topic.

## roslaunch

-**keyboard_gripper.launch**: Launch the Robotiq 2f 85 drivers and makes the gripper open or close when pressing the keyboard spacebar. Holding the spacebar will make the gripper fluctuate between the two states.
-**moveit_servo_doosan.launch**: Launch the moveit_servo real time arm servoing drivers with the Doosan config file. This enables a manipulation of the robot by using a cartesian target to where the end-effector of the robot needs to go.
-**vr_realtime.launch**: Starts the drivers of the realsense camera and of the Robotiq gripper. It also uses the scripts that act as control layer between Unity and ROS. Do not use it at the same time as the keyboard_gripper.launch.

## Example

- Launch the various ROS components needed for real-time manipulation using the following commands in multiple command window:
   ```bash
   roslaunch dsr_ros_control doosan_interface_moveit.launch # Launch the ROS driver for real-time control.
   roslauch vr_unity_ros_doosan moveit_servo_doosan_cpp.launch 
   roslaunch vr_unity_ros_doosan vr_realtime.launch # Launches camera, gripper, and conversion scripts. Permissions for the gripper may need adjustment (`sudo chmod 777 /dev/ttyUSB0`).
   roslaunch ros_tcp_endpoint endpoint.launch # Launch the Unity-ROS bridge
   rosrun rqt_controller_manager rqt_controller_manager # Switch Doosan controller to velocity control.









  
