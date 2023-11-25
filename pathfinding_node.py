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
current_path = None  # Variable to store the current path

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
            new_path = astar_dynamic(grid, start, end)

            global current_path

            if new_path:
                if current_path:
                    # Check if the new path is affected
                    if is_path_affected(new_path, grid):
                        # Recalculate the path for the affected segment
                        affected_start, affected_end = find_affected_segment(new_path, grid)

                        affected_segment = new_path[affected_start:affected_end+1]
                        new_segment = compute_path_from(grid, Node(None, affected_segment[0]), end)
                        if new_segment:
                            # Merge the paths
                            left_segment = new_path[:affected_start]
                            right_segment = new_path[affected_end+1:]
                            merged_path = left_segment + new_segment + right_segment
                            current_path = merged_path  # Update the current path
                        else:
                            rospy.loginfo("No valid path found for the affected segment.")
                    else:
                        current_path = new_path  # Update the current path
                else:
                    current_path = new_path  # Set the initial path if current path is None

            # Publish the current path
            publish_path(current_path, path_publisher)

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

