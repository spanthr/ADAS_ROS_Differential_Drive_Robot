#!/usr/bin/env python
import rospy
import math
import time
import numpy as np
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from people_msgs.msg import PositionMeasurementArray
from math import sqrt, atan2
from follow_line_step_hsv import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from darknet_ros_msgs.msg import BoundingBoxes
distanceo_1 = 1
distanceo_2 = 1
distanceo_3 = 1

def cal_average(num):
    sum_num = 0
    for t in num:
        sum_num = sum_num + t           
    if len(num) != 0:
        avg = sum_num / len(num)
        return avg

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()
        self.obstacle_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.obstacle_sub = rospy.Subscriber('/scan', LaserScan, self.call_back)
        self.rate = rospy.Rate(10)        

    def call_back(self,obs_msg):
        global laserscan
        global distanceo_1
        global distanceo_2
        global distanceo_3
        distanceo_1 = []
        distanceo_2 = []
        distanceo_3 = []
        laserscan = obs_msg

        for i,value in enumerate(laserscan.ranges):
            if (i <= 20 or i >=340):
                distanceo_1.append(value)
            if (i>20 and i<=60)  and value != float('inf'):
                distanceo_2.append(value)
            if (i>300 and i<=340)  and value != float('inf'):
                distanceo_3.append(value)

        distanceo_1 = min(distanceo_1)
        if distanceo_1 == float('inf'):
            distanceo_1 = 10
        distanceo_2 = cal_average(distanceo_2)
        distanceo_3 = cal_average(distanceo_3)


    def camera_callback(self,data):

	# We select bgr8 because its the OpneCV encoding by default
	cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            
        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        crop_img = cv_image[(height)-10:(height)][1:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = height/2, width/2
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)

        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)
        
        """
	Enter controller here.
        """
        twist_object = Twist()
        error_x = cx - width/2
        twist_object.linear.x = 0.03
        # p = 1/500
        # twist_object.angular.z = 0.01
        twist_object.angular.z = -error_x/1200
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
        # Make it start turning
        self.moveTurtlebot3_object.move_robot(twist_object)
        
    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()
        
        

def main():
    # rospy.init_node('line_following_node', anonymous=True)
    
    
    line_follower_object = LineFollower()

    
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()


def wall_obs(wall):
    global distanceo_1, distanceo_2, distanceo_3
    vel_msg = Twist()
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():

    
        if distanceo_1 < 0.2:
            vel_msg.linear.x = 0.01
            p= 1
        else:
            vel_msg.linear.x = 0.10
            p = 1
        str_angle = p*(distanceo_2 - distanceo_3)
        vel_msg.angular.z = str_angle
        wall.obstacle_pub.publish(vel_msg)
        line_follower_object.rate.sleep()


    
    
if __name__ == '__main__':
    rospy.init_node('aue_final')
    line_follower_object = LineFollower()
    main_wall(line_follower_object)
    # main()
    
    rospy.spin()
