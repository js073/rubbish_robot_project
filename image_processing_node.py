#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageProcessingNode:
    def __init__(self):
        rospy.init_node('image_processing_node', anonymous=True)

        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.red_mask = None
        self.green_mask = None

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Perform image processing
        processed_image, detection_result = self.process_image(cv_image)
        rospy.loginfo(f"Detected colors: {detection_result}")

        # Display the processed image (optional)
        cv2.imshow('Processed Image', processed_image)
        cv2.waitKey(1)

        # Extract and use the coordinates of the detected regions
        if detection_result['trash']:
            trash_coordinates = self.get_coordinates(self.red_mask)
            rospy.loginfo(f"Trash coordinates: {trash_coordinates}")
            # Implement robot movement to the trash coordinates

        if detection_result['recycling']:
            recycling_coordinates = self.get_coordinates(self.green_mask)
            rospy.loginfo(f"Recycling coordinates: {recycling_coordinates}")
            # Implement robot movement to the recycling coordinates

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

if __name__ == '__main__':
    try:
        node = ImageProcessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

