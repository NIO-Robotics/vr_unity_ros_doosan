#!/usr/bin/env python3

import rospy
import os, select, sys

from utility.robotiq import Robotiq
from robotiq_85_msgs.msg import GripperCmd, GripperStat
from std_msgs.msg import Float32

if os.name == 'nt':
    import msvcrt, time
else:
    import tty, termios


def getKey():
    """
    Taken from turtlebot3_teleop_key
    Get keyboard input
    """
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def activateGripper():
    """
    Dertermine if the gripper should open or close depending on its position.
    """
    gripper_pos = rospy.wait_for_message('/gripper/stat', GripperStat, timeout=0.1).position

    if gripper_pos > .085 / 1.5:
        gripper.close()
    else:
        gripper.open()


def keyboardGripper():
    """
    Start the node to control the gripper with a keyboard.
    """
    rospy.init_node('keyboard_gripper_node')

    global gripper 
    gripper = Robotiq()

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == ' ':
                activateGripper()
            if key == '\x03':
                break
    except:
        print("Error")

if __name__ == "__main__":
    if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

    try:
        keyboardGripper()
    except rospy.ROSInterruptException:
        pass