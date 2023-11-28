#!/usr/bin/env python


import rospy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        # Subscribe to the laser scan topic
        rospy.Subscriber('/p3dx/laser/scan', LaserScan, self.scan_callback)
        
        rospy.Subscriber('/p3dx/rrbot/camera1/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.red_mask = None
        self.green_mask = None

        # Publish twist commands to control the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Define the minimum distance for considering an object
        self.min_distance = 0.6
        self.min_area_image = 1000

    def scan_callback(self, data):
        # Process laser scan data to detect objects
        ranges = data.ranges

        # Check if there is an object within the minimum distance
        if min(ranges) < self.min_distance:
            # Stop the robot
            self.move_robot(0.0, 0.0)
            rospy.loginfo("Object Reached! Stopped.")

            # You can add more logic here to determine the direction of the object
            # and adjust the robot's movement accordingly.
        
        else:
            # Move the robot forward
            self.move_robot(0.2, 0.0)
    def image_callback(self, data):
       try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
       except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Perform image processing
       processed_image, detection_result = self.process_image(cv_image)
       rospy.loginfo(f"Detected colors: {detection_result}")

        
     

    def move_robot(self, linear_velocity, angular_velocity):
        # Create a Twist message and publish it
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)
        
    def process_image(self, image):
        # Convert BGR to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply thresholding to segment objects
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # Find contours in the thresholded image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check if there is an object with a sufficient area
        detection_result = False
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_area_image:
                detection_result = True
                break

    

        return thresh, detection_result
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Perform image processing
        processed_image, detection_result = self.process_image(cv_image)
        rospy.loginfo(f"Detected colors: {detection_result}")

  

    def process_image(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the range of red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        self.red_mask = cv2.inRange(hsv, lower_red, upper_red)

        # Define the range of green color in HSV
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        self.green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # Combine masks to detect both red and green
        combined_mask = cv2.bitwise_or(self.red_mask, self.green_mask)

        # Bitwise-AND the original image and the mask
        result = cv2.bitwise_and(image, image, mask=combined_mask)
        red_detected = cv2.countNonZero(self.red_mask)
        green_detected = cv2.countNonZero(self.green_mask)

        detection_result = {
            'trash': red_detected > 0,
            'recycling': green_detected > 0,
        }

        return result, detection_result

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


