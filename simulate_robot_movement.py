#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Point
from nav_msgs.msg import Path

# Global variable to store the path
path = []

# Callback for path updates
def path_callback(data):
    global path
    path = [(pose.pose.position.x, pose.pose.position.y) for pose in data.poses]

# Main function
def simulate_robot_movement():
    rospy.init_node('simulate_robot_movement')

    # Subscribers
    rospy.Subscriber("calculated_path", Path, path_callback)

    # Publisher
    position_publisher = rospy.Publisher("robot_position", Point, queue_size=10)

    rate = rospy.Rate(2)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        for step in path:
            position_msg = Point()
            position_msg.x = step[0]
            position_msg.y = step[1]
            position_msg.z = 0  # Assuming a 2D plane

            # Publish the robot's position
            position_publisher.publish(position_msg)

            # Log the coordinates of the step
            rospy.loginfo(f"Step taken: x={step[0]}, y={step[1]}")

            # Wait for the next step (simulate movement time)
            time.sleep(0.5)  # Adjust the sleep time as needed
            
            if step == path[-1]:
                rospy.loginfo("robot has reached the goal")
                rospy.signal_shutdown("Robot reached goal")

# Entry point for the program
if __name__ == '__main__':
    try:
        simulate_robot_movement()
    except rospy.ROSInterruptException:
        pass

