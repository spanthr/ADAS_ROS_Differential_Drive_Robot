#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

def call_back(data):
    print(data)



def rotate():
    global angular_speed
    global angle
    angular_speed = 0.2#rospy.get_param('~omega')
    angle =  3.14/2

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = angular_speed

    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    while(current_angle < angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1 - t0)
        print(current_angle)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def move_straight():
    global speed
    global distance
    speed = 0.2#rospy.get_param('~x')
    distance = 2#rospy.get_param('~length')

    vel_msg.linear.x = 0.2
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    while(current_distance < distance):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = speed*(t1 - t0)

    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    # node initialization
    rospy.init_node('go_in_circle')
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    pose_subscriber = rospy.Subscriber('pose', Pose, call_back)

    vel_msg = Twist()

    # try:
    for i in range(4):
        move_straight()
        rotate()

    #
    # except rospy.ROSInterruptExecution:
    #     pass
