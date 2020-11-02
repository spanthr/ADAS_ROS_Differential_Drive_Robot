#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    # node initialization
    rospy.init_node('go_in_circle')
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    vel_msg = Twist()

    x = rospy.get_param('~x') #0.2
    omega = rospy.get_param('~omega') #0.2

    rate = rospy.Rate(1)

    vel_msg.linear.x = x
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.z = omega
    vel_msg.angular.y = 0
    vel_msg.angular.x = 0

    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
        rate.sleep()
