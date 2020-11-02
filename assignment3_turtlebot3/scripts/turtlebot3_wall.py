#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import time

global scan_msg
global distances1, distances2, distances3

distance = 100

def call_back(msg):
    global scan_msg
    global distance

    scan_msg = msg
    distance = min(scan_msg.ranges)
    print(distance)
    # distances1 = []
    # distances2 = []
    # distances3 = []
    #
    # for i, value in enumerate(scan_msg.ranges):
    #
    #     if i <= 30 and value != float('inf'):
    #         distances1.append(value)
    #     elif i >= 330 and value != float('inf'):
    #         distances2.append(value)
    #
    # distances2.reverse()
    #
    # distances3 = distances2 + distances1 # values selected upto 30 degrees both sides.
    # if distances3 == []:
    #     distance = 100
    # else:
    #     distance = min(distances3)
    #
    # print(distance)



if __name__ == '__main__':


    rospy.init_node('turtlebot3_wall')
    scan_subscriber = rospy.Subscriber('/scan', LaserScan, call_back)
    scan_msg = LaserScan()
    vel_msg = Twist()
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)


    rate = rospy.Rate(10)

    vel_msg.linear.x = 0.2
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0




    while (distance >= 0.01):
        # print(distance)
        velocity_publisher.publish(vel_msg)
        # print(scan_msg.ranges)
        rate.sleep()


    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
