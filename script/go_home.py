#!/usr/bin/env python3

import rospy
import math
import numpy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

def calculate_joint_speed(robot_position, joint_limit):
    inner_threshold = .1 # degree
    outer_threshold = joint_limit/2
    max_speed = math.pi/20

    abs_pos = abs(robot_position)
    sign_pos = numpy.sign(robot_position)

    if abs_pos < inner_threshold:
        return 0
    elif abs_pos < outer_threshold:
        return abs_pos / outer_threshold * sign_pos * max_speed
    else:
        return sign_pos * max_speed 

def go_home():
    home_position = [0,-25,90,0,90,-90]
    joint_limit = [360, 95, 160, 360, 135, 360]
    inner_threshold = 0.003
    desired_speed = Float64MultiArray()
    desired_speed.data = [0,0,0,0,0,0]

    is_clear = 0

    while is_clear != 6:        
        is_clear = 0
        
        robot_pos = rospy.wait_for_message("/joint_states", JointState, timeout=.1)

        if robot_pos.name[0] == "joint1":
                
            for i in range(len(joint_limit)):
                joint_speed = calculate_joint_speed(home_position[i] - numpy.rad2deg(robot_pos.position[i]), joint_limit[i])
                desired_speed.data[i] = joint_speed

                if joint_speed < inner_threshold:
                    is_clear += 1
        
        pub_speed.publish(desired_speed)


if __name__ == "__main__":
    rospy.init_node("velocity_go_home_node")
    
    global pub_speed
    pub_speed = rospy.Publisher("/dsr_joint_velocity_controller/command", Float64MultiArray, queue_size=1)

    go_home()

    speed_zero = Float64MultiArray()
    speed_zero.data = [0,0,0,0,0,0]
    pub_speed.publish(speed_zero)

    try:
        print("Robot at home")
    except rospy.ROSInterruptException:
        pass
