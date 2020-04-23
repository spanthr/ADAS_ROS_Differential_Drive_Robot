#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from people_msgs.msg import PositionMeasurementArray
from math import sqrt, atan2
tags = []
dist = 0
steer = 0
person_x = 0
person_y = 0
robot_x = 0
robot_y = 0
phi = 0

def call_back(msg):
    global tags
    global person_x
    global person_y
    global phi
    tags = msg.people
    if len(tags) != 0:
        person_x = msg.people[0].pos.x
        person_y = msg.people[0].pos.y

def pose_data(msg_2):
    global robot_x
    global robot_y
    global phi 
    robot_x = msg_2.pose.pose.position.x
    robot_y = msg_2.pose.pose.position.y
    q_x = msg_2.pose.pose.orientation.x 
    q_y = msg_2.pose.pose.orientation.y 
    q_z = msg_2.pose.pose.orientation.z
    q_w = msg_2.pose.pose.orientation.w 
    K = 2*(q_w*q_z + q_x*q_y)
    L = 1 - 2*(q_y**2 + q_z**2)
    phi = np.arctan2(K, L)

if __name__ == '__main__':
    global dist
    global steer
    rospy.init_node('leg_detector', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, call_back)
    sub_robot_pose = rospy.Subscriber('/odom', Odometry, pose_data)
    rate = rospy.Rate(10)
    cmd_vel = Twist()

    while not rospy.is_shutdown():
        theta =   atan2((person_y - robot_y), (person_x - robot_x)) - phi  
        x = (person_x - robot_x)
        y = (person_y - robot_y)
        distance = sqrt(x**2 + y**2)
        print(theta, 'theta')
        print("distance front- ", distance)
        if distance < 0.2:
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
        elif distance > 0.2 and distance < 0.8:
            p_steer = 0.5
            p_dist = 0.2
            cmd_vel.linear.x = distance*p_dist
            cmd_vel.angular.z = theta*p_steer
        else:
            p_steer = 0.5
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = theta*p_steer
        pub.publish(cmd_vel)
        print(cmd_vel)

        rate.sleep()
    rospy.spin()