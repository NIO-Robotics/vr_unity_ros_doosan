#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped

'''
This script is used to tranform a Twist message from Unity
to a TwistStamped and will then be send to moveit_servo.
    Note: Moveit_servo doesn't seem to work without the timestamped which can't
          be initialized in Unity (or I didn't find how to do it).
'''

# # Global publisher to create once
# pub = rospy.Publisher('/dsr01m1013/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)
# pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)

class Unity_moveit_twist(Node):

    def __init__(self):
        super().__init__('unity_moveit_twist')
        self.publisher_=self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds',10)
        
        self.subscription = self.create_subscription(Twist, '/unity/twist', self.callback,10)

        self.get_logger().info('node started: unity_moveit_twist')
    
    def callback(self, msg: Twist):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg
        self.publisher_.publish(twist_stamped)

        

def main(args=None):
    rclpy.init(args = args)
    unity_moveit_twist = Unity_moveit_twist()
    rclpy.spin(unity_moveit_twist)
    Unity_moveit_twist.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()