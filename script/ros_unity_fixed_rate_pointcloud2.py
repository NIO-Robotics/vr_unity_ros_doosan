#!/usr/bin/env python3

import rospy
from rospy import Publisher
from sensor_msgs.msg import PointCloud2

pub = rospy.Publisher('/ros/unity/fixed_rate_pointcloud2', PointCloud2,queue_size=1)

counter = 0

def callback(msg):
    """ 
    Get the message of a topic and publish it at a fixed frequence
    :param msg: Depth perception of a camera
    :type msg: Pointcloud2
    """
    global counter
    counter += 1
    
    if counter%20 == 0:
        pub.publish(msg)

def fix_rate_pointcloud2():
    """
    Subscribe to a pointcloud2 topic to republish it at a lower frequence
    """

    rospy.init_node('fixed_rate_pointcloud2')
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback, queue_size=1) 
    #rospy.Subscriber("/voxel_grid/output", PointCloud2, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    fix_rate_pointcloud2()
