#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray

# Initialize the ROS node
rospy.init_node('grid_update_publisher')

# Create a publisher
grid_publisher = rospy.Publisher('grid_update', Int32MultiArray, queue_size=10)

rate = rospy.Rate(1)  # 1 Hz

while not rospy.is_shutdown():
    # Create an Int32MultiArray message
    grid_msg = Int32MultiArray()

    # Define a 6x6 grid with no obstacles (all zeros)
    # Flatten the grid to a one-dimensional list
    grid_msg.data = [0 for _ in range(36)]  # 6x6 grid, all zeros

    # Publish the message
    grid_publisher.publish(grid_msg)

    rate.sleep()

