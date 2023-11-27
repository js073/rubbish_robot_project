#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

# Global variable to store the current position and orientation of the robot
current_position = [0, 0]  # Assuming initial position (x, y)
current_orientation = 'up'  # Assuming initial orientation (up, down, left, right)

def get_rotation_direction(current_orientation, target_orientation):
    # Map of orientations to their left and right rotations
    orientations = {
        'up': {'left': 'left', 'right': 'right'},
        'down': {'left': 'right', 'right': 'left'},
        'left': {'left': 'down', 'right': 'up'},
        'right': {'left': 'up', 'right': 'down'}
    }

    if orientations[current_orientation]['left'] == target_orientation:
        return 'left'
    elif orientations[current_orientation]['right'] == target_orientation:
        return 'right'
    else:
        return 'none'  # No rotation needed

def calculate_orientation_change(current_position, target_position):
    if target_position[0] < current_position[0]:
        return 'left'
    elif target_position[0] > current_position[0]:
        return 'right'
    elif target_position[1] < current_position[1]:
        return 'down'
    else:
        return 'up'

def simulate_movement_to_target(target_x, target_y):
    global current_position, current_orientation

    target_orientation = calculate_orientation_change(current_position, [target_x, target_y])
    rotation_direction = get_rotation_direction(current_orientation, target_orientation)

    cmd_vel_msg = Twist()

    # Rotation logic
    if rotation_direction != 'none':
        cmd_vel_msg.linear.x = 0.0  # Stop linear movement for rotation
        cmd_vel_msg.angular.z = 0.5 if rotation_direction == 'left' else -0.5  # Positive for left, negative for right
        current_orientation = target_orientation  # Update orientation after rotation
    else:
        # Straight movement
        cmd_vel_msg.linear.x = 0.05  # Move one cell (0.05 meters) in 1 second
        cmd_vel_msg.angular.z = 0.0  # No rotation for straight movement
        current_position = [target_x, target_y]  # Update position after movement

    return cmd_vel_msg

def detailed_movement_callback(data):
    target_x = data.x
    target_y = data.y

    cmd_vel_msg = simulate_movement_to_target(target_x, target_y)
    
    duration = rospy.Duration(1.0)  # Movement duration
    start_time = rospy.Time.now()

    while rospy.Time.now() - start_time < duration:
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()

    # Publish a message to indicate movement completion
    movement_completed_pub.publish(Bool(True))

def simulate_robot_movement():
    rospy.init_node('simulate_robot_movement')

    # Publisher for indicating movement completion
    movement_completed_pub = rospy.Publisher("/robot_movement_completed", Bool, queue_size=1)

    # Publisher for sending velocity commands
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Subscriber for detailed movement requests
    rospy.Subscriber("/detailed_robot_movement_request", Point, detailed_movement_callback)

    # Set rate for the loop
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        rate.sleep()

# Entry point for the program
if __name__ == '__main__':
    try:
        simulate_robot_movement()
    except rospy.ROSInterruptException:
        pass

