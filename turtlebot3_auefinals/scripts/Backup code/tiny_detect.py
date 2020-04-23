#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
global distance_1
global distance_2
global distance_3
global laserscan
distance_1 = 0.5
distance_2 = 1
distance_3 = 1
num = 0

def call_back(msg):
    print(type(msg.poses))
    # global num
    # num=msg

if __name__ == '__main__':
    rospy.init_node('stop_detector')
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_people = rospy.Subscriber("/to_pose_array/leg_detector",PoseArray, call_back)
    vel_msg = Twist()
    rate = rospy.Rate(10)
    

    while not rospy.is_shutdown():
        # print(num)
        # if num.bounding_boxes[len(num.bounding_boxes)- 1].id == 11:
        #     vel_msg.linear.x = 0.0
        rate.sleep()