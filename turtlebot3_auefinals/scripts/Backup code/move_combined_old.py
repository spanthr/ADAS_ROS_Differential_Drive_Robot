#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
global distanceo_1
global distanceo_2
global distanceo_3
distanceo_1 = 0.5
distanceo_2 = 1
distanceo_3 = 1
global laserscan
distw_1 = 1
distw_2 = 1


def cal_average(num):
    sum_num = 0
    for t in num:
        sum_num = sum_num + t           

    avg = sum_num / len(num)
    return avg


def call_back_wall(wall_msg):
    global laserscan
    global distw_1
    global distw_2
    distancew_1 = []
    distancew_2 = []
    laserscan = wall_msg
    

    for i,value in enumerate(laserscan.ranges):
        if (i>=30 and i<=90) and value != float('inf'):
            distancew_1.append(value)
        if (i>=270 and i<=330) and value != float('inf'):
            distancew_2.append(value)

    if len(distancew_1) == 0:
        distw_1 = 1
    if len(distancew_2) == 0:
        distw_2 = 1
    if len(distancew_1) != 0 and len(distancew_2) != 0:
        distw_1 = cal_average(distancew_1)
        distw_2 = cal_average(distancew_2)

def call_back_obs(obs_msg):
    global laserscan
    global distanceo_1
    global distanceo_2
    global distanceo_3
    distanceo_1 = []
    distanceo_2 = []
    distanceo_3 = []
    laserscan = obs_msg
    

    for i,value in enumerate(laserscan.ranges):
        if (i <= 20 or i >=340) and value != float('inf'):
            distanceo_1.append(value)
        if (i>20 and i<=50) and value != float('inf'):
            distanceo_2.append(value)
        if (i>310 and i<=340) and value != float('inf'):
            distanceo_3.append(value)

    distanceo_1 = min(distanceo_1)
    print(distanceo_1)
    # distanceo_2 = min(distanceo_2)
    distanceo_2 = cal_average(distanceo_2)
    # distanceo_3 = min(distanceo_3)
    distanceo_3 = cal_average(distanceo_3)
    

def obstacle_avoidance():
    
    if distanceo_1 < 0.2:
        vel_msg.linear.x = 0.01
        p= 0.9
    else:
        vel_msg.linear.x = 0.10
        p = 1
    str_angle = p*(distanceo_2 - distanceo_3)
    vel_msg.angular.z = str_angle
    # if distanceo_1 <= 0.2:
    #     vel_msg.linear.x = 0.01
    #     vel_msg.angular.z = 0.2
    
    velocity_publisher.publish(vel_msg)
    # print(vel_msg)

def wall_following():
    vel_msg.linear.x = 0.15
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    p= 0.5
    str_angle = p*(distw_1 - distw_2)
    vel_msg.angular.z = str_angle
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('wall_following_code')
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    follower_subscriber_wall = rospy.Subscriber('/scan' , LaserScan, call_back_wall)
    follower_subscriber_obs = rospy.Subscriber('/scan' , LaserScan, call_back_obs)
    laserscan = LaserScan()
    vel_msg = Twist()
   
    rate = rospy.Rate(10)
    

while not rospy.is_shutdown():
    # wall_following()
    obstacle_avoidance()
   
    rate.sleep()