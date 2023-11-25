#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from astar_dynamic import astar_dynamic, Node, is_path_affected, compute_path_from, merge_paths

# Global variables to store the grid and the robot's position
grid = None
robot_position = None


# Callback for robot position updates
def robot_position_callback(data):
    global robot_position
    robot_position = (data.x, data.y)

# Callback for grid updates
def grid_update_callback(data):
    global grid
    # Assuming the grid data is in the form of a list representing a 2D grid
    grid = [data.data[i:i+6] for i in range(0, len(data.data), 6)]

# Main function
def pathfinding_node():
    rospy.init_node('pathfinding_node')

    # Subscribers
    rospy.Subscriber("robot_position", Point, robot_position_callback)
    rospy.Subscriber("grid_update", Int32MultiArray, grid_update_callback)

    # Publisher
    path_publisher = rospy.Publisher("calculated_path", Path, queue_size=10)

    # Set rate for the loop
    rate = rospy.Rate(10)  # 10 Hz

    # Main loop
    while not rospy.is_shutdown():
        if grid is not None and robot_position is not None:
            # Define start and end points for A* algorithm
            start = robot_position  # Assuming robot_position is a tuple (x, y)
            end = (5, 5)  # Example goal

            # Call your A* dynamic function here
            path = astar_dynamic(grid, start, end)

            if path:
                # Publish the initial path
                publish_path(path, path_publisher)

                # Check if path is affected at each step
                for index, step in enumerate(path):
                    if is_path_affected(path[index:], grid):
                        affected_segment_start = index
                        new_path_segment = compute_path_from(grid, Node(None, step), end)
                        if new_path_segment:
                            path = merge_paths(new_path_segment, path[:affected_segment_start])
                            publish_path(path, path_publisher)
                        else:
                            rospy.loginfo("No valid path found from current position.")
                            break

        rate.sleep()

def publish_path(path, publisher):
    path_msg = Path()
    path_msg.header.frame_id = "map"  # Set the appropriate frame ID
    for step in path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = step[0]
        pose.pose.position.y = step[1]
        pose.pose.position.z = 0
        pose.pose.orientation.w = 1  # Neutral orientation
        path_msg.poses.append(pose)

    publisher.publish(path_msg)

# Entry point for the program
if __name__ == '__main__':
    try:
        pathfinding_node()
    except rospy.ROSInterruptException:
        pass

