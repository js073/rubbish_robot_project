#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from std_msgs.msg import Bool
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from astar_dynamic import astar_dynamic

# Global variables to store the grid, robot's position, and path
grid = None
robot_position = None
current_path = None  # Variable to store the current path
current_step = 0  # Variable to track the current step in the path

# Define the goal position (change this to your goal)
goal = (2005, 2007)

# Threshold for checking if the robot has reached its target
POSE_THRESHOLD = 0.025

# Function to convert world coordinates to grid indices
def world_to_grid(world_x, world_y, cell_size, grid_width, grid_height):
    grid_x = int((world_x / cell_size) + (grid_width / 2))
    grid_y = int((world_y / cell_size) + (grid_height / 2))
    return grid_x, grid_y

# Callback for robot position updates from /amcl_pose
def robot_position_callback(data):
    global robot_position
    robot_position = (data.pose.pose.position.x, data.pose.pose.position.y)

# Callback for map updates
def map_callback(data):
    global grid
    grid = [data.data[i:i+4000] for i in range(0, len(data.data), 4000)]
    
            
# Function to check if the robot is close to its target
def is_close_to_target(target):
    global robot_position
    dx = robot_position[0] - target[0]
    dy = robot_position[1] - target[1]
    distance = (dx**2 + dy**2)**0.5
    rospy.loginfo("Distance to target: {:.2f}, Threshold: {:.2f}".format(distance, POSE_THRESHOLD))
    return distance < POSE_THRESHOLD

# Publishers
detailed_movement_pub = rospy.Publisher("/detailed_robot_movement_request", Point, queue_size=1)
robot_movement_completed_pub = rospy.Publisher("/robot_movement_completed", Bool, queue_size=1)

# Main function
def pathfinding_node():
    rospy.init_node('pathfinding_node')
    rospy.loginfo("Pathfinding node started")

    # Subscribers
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, robot_position_callback)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)

    # Loop rate
    rate = rospy.Rate(10)  # 10 Hz

    cell_size = 0.05  # meters per cell
    grid_width = 4000
    grid_height = 4000

    while not rospy.is_shutdown():
        if grid is not None and robot_position is not None:
            global current_step, current_path

            # Initial path calculation
            if current_path is None:
                rospy.loginfo("Calculating new path")
                start = world_to_grid(robot_position[0], robot_position[1], cell_size, grid_width, grid_height)
                end = goal
                rospy.loginfo("Start: {}, End: {}".format(start, end))
                new_path = astar_dynamic(grid, start, end)
                rospy.loginfo(str(new_path))

                if new_path:
                    current_path = new_path
                    current_step = 0  # Reset the current step

            # Process next step in the path
            if current_path and current_step < len(current_path) - 1:
                rospy.loginfo("Processing next step in the current path")
                next_cell = current_path[current_step + 1]

                # Move to the next cell
                detailed_movement_pub.publish(Point(x=next_cell[0], y=next_cell[1], z=0))
                rospy.wait_for_message("/robot_movement_completed", Bool)

                if is_close_to_target(next_cell):
                    current_step += 1

        rate.sleep()

    rospy.loginfo("Shutting down pathfinding node")

# Entry point for the program
if __name__ == '__main__':
    try:
        pathfinding_node()
    except rospy.ROSInterruptException:
        pass
