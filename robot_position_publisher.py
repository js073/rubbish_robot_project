#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

# Callback function to update robot position
def update_robot_position(data):
    global position
    position.x = data.pose.pose.position.x
    position.y = data.pose.pose.position.y
    # position.z is not used for 2D plane

# Initialize the ROS node
rospy.init_node('robot_position_publisher')

# Create a global position variable and initialize it to (0,0)
position = Point()
position.x = 0
position.y = 0
position.z = 0  # Assuming a 2D plane

# Subscribe to the robot's odometry (or pose) topic
odom_subscriber = rospy.Subscriber('/robot_name/odom', Odometry, update_robot_position)  # Change topic and message type if needed

# Create a publisher
position_publisher = rospy.Publisher('robot_position', Point, queue_size=10)

rate = rospy.Rate(10)  # 10 Hz

while not rospy.is_shutdown():
    # Publish the updated position
    position_publisher.publish(position)
    rate.sleep()

