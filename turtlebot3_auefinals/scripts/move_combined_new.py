#!/usr/bin/env python

#!/usr/bin/env python
# Team-2 Final Project Script
# Autonomy: Systems and Sciences
# importing the libraries
import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from people_msgs.msg import PositionMeasurementArray
from math import sqrt, atan2
from darknet_ros_msgs.msg import BoundingBoxes
import time 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from apriltag_ros.msg import AprilTagDetectionArray
import cv2

# initilization of variables
distanceo_1 = 1
distanceo_2 = 1
distanceo_3 = 1
dist = 0
steer = 0
person_x = 0
person_y = 0
robot_x = 0
robot_y = 0
phi = 0
num = 0
count = 0
tags = []
code = 0
line_follower_object = 0
num = 0
loop_once = 1
flag = 0

class LineFollower(object):
    
    def __init__(self): # Class constructor for line following
        global flag
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.stop_sign_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes' , BoundingBoxes, self.stop_sign_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()

    def stop_sign_callback(self,msg): # Callback function for stop sign detection
        global num
        num = 0
        if msg.bounding_boxes[len(msg.bounding_boxes)- 1].id == 11:
            num = 1
        

    def camera_callback(self,data): # Callback fuction for camera data

	# We select bgr8 because its the OpneCV encoding by default
	cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            
        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        crop_img = cv_image[(height/2) + 100:(height/2) + 120][1:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        global flag
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            flag = 0 # Assignment of flag as zero if the yellow line is detected
        except ZeroDivisionError:
            cx, cy = height/2, width/2
            flag = 1 # Assignment of flag as one if the yellow line is not detected.

        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)

        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)
        
        """
	Enter controller here.
        """
        if flag == 0: # Following the yellow line
            twist_object = Twist()
            error_x = cx - width/2
            twist_object.linear.x = 0.08
            twist_object.angular.z = -error_x/1400
            self.moveTurtlebot3_object.move_robot(twist_object)
            global num
            global loop_once
            if num == 1:
                if loop_once == 1: # Stop sign detection
                    twist_object.linear.x = 0
                    twist_object.angular.z = 0
                    self.moveTurtlebot3_object.move_robot(twist_object)
                    rospy.sleep(3)
                    num = 0
                    loop_once = 0
        if flag == 1: # Following the human
            global linear_x
            global angular_z
            linear_x, angular_z = leg_detect()
            twist_object = Twist()
            twist_object.linear.x = linear_x
            twist_object.angular.z = angular_z
            self.moveTurtlebot3_object.move_robot(twist_object)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def cal_average(num): # function to calculate average of the input list
    sum_num = 0
    for t in num:
        sum_num = sum_num + t           
    if len(num) != 0:
        avg = sum_num / len(num)
        return avg

def april_tag_callback(april_tag): # Call back function for april tag detection
    global tags
    global code
    tags = []
    tags = april_tag.detections
    if len(tags) != 0 and tags[0].id[0] == 1: # April tag with ID = 1 is used.
            if tags[0].pose.pose.pose.position.z < 0.2:
                code = 1

def call_back_leg_detector(msg): # Call back function for leg detector
    global person_x
    global person_y
    if len(msg.poses) != 0:
        person_x = msg.poses[0].position.x
        person_y = msg.poses[0].position.y
    
def pose_data(msg_2):# Call back for the position and orientation of the robot
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

def call_back_obs(obs_msg): # Call back for obstacle avoidance and wall following.
    global laserscan
    global distanceo_1 # list of front distance readings.
    global distanceo_2 # list of left side distance readings.
    global distanceo_3 # list of right side distance readings.
    distanceo_1 = []
    distanceo_2 = []
    distanceo_3 = []
    laserscan = obs_msg

    for i,value in enumerate(laserscan.ranges):
        if (i <= 20 or i >=340):
            distanceo_1.append(value) # List of front 40 degree lidar scan data.
        if (i>20 and i<=70)  and value != float('inf'):
            distanceo_2.append(value) # List of 40 degree lidar scan data on the left side of the turtlebot.
        if (i>290 and i<=340)  and value != float('inf'):
            distanceo_3.append(value) # List of 40 degree lidar scan data on the right side of the turtlebot.

    distanceo_1 = min(distanceo_1)
    if distanceo_1 == float('inf'):
        distanceo_1 = 10
    if len(distanceo_2) != 0:
        distanceo_2 = cal_average(distanceo_2) # average values of the left hand distance readings
    else:
        distanceo_2 = 1
    if len(distanceo_3) != 0:
        distanceo_3 = cal_average(distanceo_3)# average values of the right hand distance readings
    else:
        distanceo_3 = 1
    
def obstacle_avoidance(): # obstacle avoidance function
    
    if distanceo_1 < 0.25:
        vel_msg.linear.x = 0.01
        p= 2
    else:
        vel_msg.linear.x = 0.20
        p = 2.5
    str_angle = p*(distanceo_2 - distanceo_3)
    vel_msg.angular.z = str_angle
    velocity_publisher.publish(vel_msg) # publishing the velocity to the turtlebot

def leg_detect(): # function for leg detection
    sub_people = rospy.Subscriber("/to_pose_array/leg_detector",PoseArray, call_back_leg_detector)
    sub_robot_pose = rospy.Subscriber('/odom', Odometry, pose_data) # Subscriber of the robot's pose.
    angle = atan2((person_y - robot_y), (person_x - robot_x))
    if angle < 0:
        angle = 2*np.pi - abs(angle)
    theta = angle - phi # Angular difference between the human's global orientation and the turtlebot's orientation
    x = (person_x - robot_x)
    y = (person_y - robot_y)
    distance = sqrt(x**2 + y**2) # Euclideandistance between human and turtlebot

    if distance < 0.28:
        linear_x = 0
        angular_z = 0
    elif distance > 0.28 :
        p_steer = 0.5
        p_dist = 0.2
        linear_x = distance*p_dist
        angular_z = theta*p_steer
    return linear_x, angular_z

def line_following():  # Function for line following and person detection
    global line_follower_object
    line_follower_object = LineFollower() # Object of the Line Follower class

    
    ctrl_c = False
    def shutdownhook(): # Function to quit the loop
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)

if __name__ == '__main__':
    # global tags
    rospy.init_node('move_combined_code') # Node initilization
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Velocity Publisher for wall following and obstacle avoidance tasks.
    follower_subscriber_obs = rospy.Subscriber('/scan' , LaserScan, call_back_obs) # Subscriber for wall following and obstacle avoidance tasks.
    tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, april_tag_callback) # Subscriber for april tag detection.
    vel_msg = Twist()
    global rate
    rate = rospy.Rate(10)
    
    while code == 0: # variable 'code' is zero when april-tag has not been detected.
        obstacle_avoidance() # Calling the obstacle avoidance function
        rate.sleep()
    if code == 1: # April tag has been detected
        if flag == 0:# Flag is zero when line is visible.
            line_following()# Calling the line-following function once the april tag is detected.
    rospy.spin()