#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

# Global variables to store the current position and orientation of the robot
current_position = [0, 0]  # Assuming initial position (x, y)
current_orientation = 'up'  # Assuming initial orientation (up, down, left, right)
cmd_vel_pub = None

# Movement parameters
MOVE_DISTANCE = 0.05  # meters per cell
MOVE_DURATION = 1.0   # seconds per move
TURN_SPEED = 1.57     # rad/s, assuming 90-degree turn in 1 second

def determine_orientation(current_x, current_y, target_x, target_y):
    dx = target_x - current_x
    dy = target_y - current_y
    rospy.loginfo(f"Determining orientation: Current ({current_x}, {current_y}), Target ({target_x}, {target_y})")
    if abs(dx) > abs(dy):
        # Movement is primarily along the X-axis
        return 'right' if dx > 0 else 'left'
    else:
        # Movement is primarily along the Y-axis
        return 'up' if dy > 0 else 'down'

def calculate_movement(target_x, target_y):
    global current_position, current_orientation

    dx = target_x - current_position[0]
    dy = target_y - current_position[1]

    cmd_vel_msg = Twist()

    # Determine target orientation based on movement direction
    target_orientation = determine_orientation(current_position[0], current_position[1], target_x, target_y)

    # Rotate to face the correct direction if needed
    if current_orientation != target_orientation:
        if (current_orientation, target_orientation) in [('up', 'right'), ('right', 'down'), ('down', 'left'), ('left', 'up')]:
            cmd_vel_msg.angular.z = -TURN_SPEED  # Rotate clockwise
        else:
            cmd_vel_msg.angular.z = TURN_SPEED   # Rotate counter-clockwise
        rospy.sleep(1)  # Wait for 1 second to complete rotation

    # Move forward to the next cell
    cmd_vel_msg.angular.z = 0  # Stop rotating
    cmd_vel_msg.linear.x = MOVE_DISTANCE / MOVE_DURATION  # Speed to move one cell in 1 second
    rospy.sleep(1)  # Wait for 1 second to complete movement

    # Update current orientation and position
    current_orientation = target_orientation
    current_position = [target_x, target_y]
    rospy.loginfo(f"Calculated movement to ({target_x}, {target_y}), Orientation: {target_orientation}")

    return cmd_vel_msg

def detailed_movement_callback(data):
    global current_position, current_orientation
    target_x = data.x
    target_y = data.y

    cmd_vel_msg = calculate_movement(target_x, target_y)

    # Publish the movement command
    cmd_vel_pub.publish(cmd_vel_msg)

    current_position = [target_x, target_y]
    current_orientation = determine_orientation(current_position[0], current_position[1], target_x, target_y)

    # Publish movement completion
    robot_movement_completed_pub.publish(Bool(True))
    rospy.loginfo(f"Received movement request to ({data.x}, {data.y})")
    rospy.loginfo(f"Publishing movement command: Linear {cmd_vel_msg.linear.x}, Angular {cmd_vel_msg.angular.z}")
    rospy.loginfo(f"Updated position to ({current_position[0]}, {current_position[1]}), Orientation: {current_orientation}")

def simulate_robot_movement():
    global cmd_vel_pub, robot_movement_completed_pub
    rospy.init_node('simulate_robot_movement')
    rospy.loginfo("Simulate robot movement node started")

    # Publishers
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    robot_movement_completed_pub = rospy.Publisher("/robot_movement_completed", Bool, queue_size=1)

    # Subscriber for detailed movement requests
    rospy.Subscriber("/detailed_robot_movement_request", Point, detailed_movement_callback)

    rospy.spin()
    rospy.loginfo("simulate_robot_movement node is shutting down")

# Entry point for the program
if __name__ == '__main__':
    try:
        simulate_robot_movement()
    except rospy.ROSInterruptException:
        rospy.logerr("simulate_robot_movement node interrupted and shut down")
