#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
PI=3.1415926535897

def move():
    speed = 0.2
    distance = 2

    vel_msg.linear.x = speed
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

def rot():
    angular_speed = 0.2
    angle = 90*PI/180

    vel_msg.linear.x = 0

    vel_msg.angular.z = angular_speed

    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    while(current_angle < angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1 - t0)
    vel_msg.linear.z = 0
    velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        #Testing our function
        rospy.init_node('square_openloop')
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
        vel_msg = Twist()
        for side in range(4):
            move()
            rot()
    except rospy.ROSInterruptException:
        pass
