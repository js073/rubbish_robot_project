#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from std_msgs.msg import Int32MultiArray, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from astar_dynamic import astar_dynamic, is_path_affected, find_affected_segment

# Global variables to store the grid, robot's position, and path
grid = None
robot_position = None
current_path = None  # Variable to store the current path
current_step = 0  # Variable to track the current step in the path

# Define the goal position (change this to your goal)
goal = (5, 5)

# Callback for robot position updates from /amcl_pose
def robot_position_callback(data):
    global robot_position
    robot_position = (data.pose.pose.position.x, data.pose.pose.position.y)

# Callback for grid updates
def grid_update_callback(data):
    global grid
    # Assuming the grid data is in the form of a list representing a 2D grid
    grid = [data.data[i:i+6] for i in range(0, len(data.data), 6)]

# Publisher for detailed movement commands
detailed_movement_pub = rospy.Publisher("/detailed_robot_movement_request", Point, queue_size=1)

# Publisher for indicating movement completion
robot_movement_completed_pub = rospy.Publisher("/robot_movement_completed", Bool, queue_size=1)

# Main function
def pathfinding_node():
    rospy.init_node('pathfinding_node')

    # Subscribers
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, robot_position_callback)
    rospy.Subscriber("grid_update", OccupancyGrid, grid_update_callback)

    # Publisher
    path_publisher = rospy.Publisher("calculated_path", Path, queue_size=10)

    # Set rate for the loop
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if grid is not None and robot_position is not None:
            global current_step, current_path

            # Step 1: Initial path calculation
            if current_path is None:
                start = robot_position
                end = goal

                new_path = astar_dynamic(grid, start, end)

                if new_path:
                    current_path = new_path
                    current_step = 0  # Reset the current step

            # Step 2: Check if the path is still valid
            if current_path is not None and current_step < len(current_path):
                next_cell = current_path[current_step]
                start = robot_position

                if not is_path_affected(current_path, grid):
                    # Publish the next cell coordinates
                    detailed_movement_pub.publish(Point(x=next_cell[0], y=next_cell[1], z=0))
                    # Wait for robot movement completion
                    rospy.wait_for_message("/robot_movement_completed", Bool)
                    current_step += 1
                else:
                    affected_start, affected_end = find_affected_segment(current_path, grid)

                    if affected_start is not None:
                        # Recalculate the affected segment
                        affected_segment = current_path[affected_start:affected_end + 1]
                        new_path_segment = astar_dynamic(grid, start, affected_segment[-1])

                        if new_path_segment:
                            # Merge the paths: left + new affected segment + right
                            current_path = current_path[:affected_start] + new_path_segment + current_path[affected_end + 1:]
                            current_step = affected_start + len(new_path_segment)  # Update the current step
                            publish_path(current_path, path_publisher, current_step)
                            # Publish the next cell coordinates
                            next_cell = current_path[current_step]
                            detailed_movement_pub.publish(Point(x=next_cell[0], y=next_cell[1], z=0))
                            # Wait for robot movement completion
                            rospy.wait_for_message("/robot_movement_completed", Bool)

        rate.sleep()

def publish_path(path, publisher, step):
    path_msg = Path()
    path_msg.header.frame_id = "map"  # Set the appropriate frame ID
    for i in range(step + 1):  # Publish up to the current step
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = path[i][0]
        pose.pose.position.y = path[i][1]
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
