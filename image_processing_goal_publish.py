#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
import gazebo_services

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
        self.status_pub = rospy.Publisher('/task/commands', String, queue_size=100)

        # Define the minimum distance for considering an object
        self.min_distance = 0.3
        self.min_area_image = 1000
        self.detected = None
        self.can_move = True

    def scan_callback(self, data):
        # Process laser scan data to detect objects
        ranges = data.ranges
        ranges = self.get_used_ranges(data)

        if self.can_move:
            # Check if there is an object within the minimum distance
            if min(ranges) < self.min_distance:
                # Stop the robot
                self.move_robot(0.0, 0.0)
                rospy.loginfo("Object Reached! Stopped.")
                gazebo_services.remove_nearest_rubbish()

                # You can add more logic here to determine the direction of the object
                # and adjust the robot's movement accordingly.

    
    def image_callback(self, data):
        try:
                cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
                rospy.logerr(e)
                return

            # Perform image processing
        processed_image, detection_result = self.process_image(cv_image)
        self.detected = detection_result
        rospy.loginfo(f"Detected colors: {detection_result}")

        s = String()
        s.data = "pause" if detection_result['trash'] or detection_result['recycling'] else "resume"
        self.status_pub.publish(s)
        self.can_move = detection_result['trash'] or detection_result['recycling']

        if detection_result['trash']:
            trash_coordinates = self.get_coordinates(self.red_mask)
            rospy.loginfo(f"Trash coordinates: {trash_coordinates}")
            self.decide_robot_movement(trash_coordinates[0])
            # Implement robot movement to the trash coordinates

        if detection_result['recycling']:
            recycling_coordinates = self.get_coordinates(self.green_mask)
            rospy.loginfo(f"Recycling coordinates: {recycling_coordinates}")
            self.decide_robot_movement(recycling_coordinates[0])
            # Implement robot movement to the recycling coordinates

    def decide_robot_movement(self, coords):
        (x, y) = coords
        if 700 < x < 900:
            self.move_robot(0.2, 0)
        elif x < 700:
            self.move_robot(0, 0.5)
        else: 
            self.move_robot(0, -0.5)

    def move_robot(self, linear_velocity, angular_velocity):
        # Create a Twist message and publish it
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)
        
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
    
    def get_coordinates(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        coordinates = []

        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                coordinates.append((cX, cY))

        return coordinates
    
    def get_used_ranges(self, scan: LaserScan):
        increment = scan.angle_increment
        fov = (10 / 180) * math.pi
        number_increments = int((fov / increment) / 2)
        scans = scan.ranges
        mid_point = int(len(scans) / 2)
        new_scans = scans[(mid_point - number_increments):(mid_point + number_increments)]
        return new_scans
        



if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


