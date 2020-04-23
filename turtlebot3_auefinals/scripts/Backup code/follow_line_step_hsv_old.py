#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from darknet_ros_msgs.msg import BoundingBoxes
global detect_time
detect_time = float('inf')

class LineFollower(object):

    def __init__(self):
        print("jafbahkm")
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.boundingbox_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes' , BoundingBoxes, self.call_back)
        self.moveTurtlebot3_object = MoveTurtlebot3()
    
    def call_back(self,msg):
        global num
        num = 0
        if msg.bounding_boxes[len(msg.bounding_boxes)- 1].id == 11 and num == 0:
            num = 1
            detect_time = rospy.Time.now().to_sec()


    
    def camera_callback(self,data):

	# We select bgr8 because its the OpneCV encoding by default
	cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

            
        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        crop_img = cv_image[(height) - 10 :(height) ][1:width]
        
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
        twist_object.linear.x = 0.05
        # p = 1/500
        # twist_object.angular.z = 0.01
        twist_object.angular.z = -error_x/1050
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
        # Make it start turning

        now_time = rospy.Time.now().to_sec()
        if (detect_time + 3 < now_time):
            twist_object.linear.x = 0.0
            twist_object.angular.z = 0.0
        print("now time",now_time)
        print("detect_time",detect_time)
        self.moveTurtlebot3_object.move_robot(twist_object)
    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()
        
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    
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

    
    
if __name__ == '__main__':
    main()
