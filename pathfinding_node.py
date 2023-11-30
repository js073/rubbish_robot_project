#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Pose, PoseArray
from std_msgs.msg import Bool, String, Time, Float32
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from astar_dynamic import astar_dynamic, is_path_affected, find_affected_segment
import map_inflation_inv

# Global variables to store the grid, robot's position, and path
grid = None
robot_position = None
current_path = None  # Variable to store the current path
current_step = 0  # Variable to track the current step in the path
running = True
new_path_needed = True

# Define the goal position (change this to your goal)
goal: PoseStamped = None

# Threshold for checking if the robot has reached its target
POSE_THRESHOLD = 0.025

# Function to convert world coordinates to grid indices
def world_to_grid(world_x, world_y, cell_size, grid_width, grid_height):
    grid_x = int((world_x / cell_size) + (grid_width / 2))
    grid_y = int((world_y / cell_size) + (grid_height / 2))
    return grid_x, grid_y

# Callback for robot position updates from /amcl_pose
def robot_position_callback(data: Odometry):
    global robot_position
    robot_position = (data.pose.pose.position.x, data.pose.pose.position.y)

# Callback for map updates
def map_callback(data):
    global grid
    grid_height = data.info.height
    grid_width = data.info.width

    grid = map_inflation_inv.inflate_map(data)
    return

    # Create a grid of dimensions [grid_width x grid_height]
    temp = [[0 for _ in range(grid_height)] for _ in range(grid_width)]

    # Fill the grid
    for i in range(len(data.data)):
        grid_x = i % grid_width
        grid_y = i // grid_width
        temp[grid_x][grid_y] = data.data[i]
    grid = temp

# Function to check if the robot is close to its target
def is_close_to_target(target):
    global robot_position
    dx = robot_position[0] - target[0]
    dy = robot_position[1] - target[1]
    distance = (dx ** 2 + dy ** 2) ** 0.5
    rospy.loginfo(str(target))
    rospy.loginfo("Distance to target: {:.2f}, Threshold: {:.2f}".format(distance, POSE_THRESHOLD))
    return distance < POSE_THRESHOLD

# Function to check if the robot is close to its target
def is_close_to_target(target):
    global robot_position
    dx = robot_position[0] - target[0]
    dy = robot_position[1] - target[1]
    distance = (dx**2 + dy**2)**0.5
    rospy.loginfo(str(target))
    rospy.loginfo("Distance to target: {:.2f}, Threshold: {:.2f}".format(distance, POSE_THRESHOLD))
    return distance < POSE_THRESHOLD

def get_diag(p, i):
    start = p[i]
    end = p[i]
    j = i + 2
    while j < len(p):
        current = p[j]
        d_x = abs(current[0] - start[0])
        d_y = abs(current[1] - start[1])
        if d_x == d_y:
            end = current
            j += 2
        else:
            return start, end, ((j-2)-i)
    return start, end, ((j-2)-i)

def get_straight(p, i):
    first = p[i]
    print("first")
    prev = p[i]
    j = i
    for j in range(i + 1, len(p)):
        current = p[j]
        print(current)
        if first[0] - current[0] == 0 or first[1] - current[1] == 0:
            prev = current
        else:
            return first, prev, ((j-1)-i)
    return first, prev, ((j)-i)

def smooth_current_path(path):
    new_path = []
    i = 0
    added = False
    if len(path) > 2:
        while i < len(path):
            (s, e, inc) = get_diag(path, i)
            if inc != 0:
                if not(added):
                    new_path.append(s)
                new_path.append(e)
                added = True
                i += inc
                continue
            (s, e, inc) = get_straight(path, i)
            if inc != 0:
                if not(added):
                    new_path.append(s)
                new_path.append(e)
                added = True
                i += inc
                continue
            if not(added):
                new_path.append(path[i])
            i += 1 

            
        # new_path.append(path[-2])
        # new_path.append(path[-1])
        return new_path
    else:
        return path
    
def status_callback(msg: String):
    global running
    global new_path_needed
    s = msg.data
    if "pause" in s:
        running = False
    elif "resume" in s:
        if not(running):
            new_path_needed = True
        running = True



# Publishers
detailed_movement_pub = rospy.Publisher("/detailed_robot_movement_request", Point, queue_size=1)
robot_movement_completed_pub = rospy.Publisher("/robot_movement_completed", Float32, queue_size=1)

# Main function
def pathfinding_node():
    rospy.init_node('pathfinding_node')
    rospy.loginfo("Pathfinding node started")

    # Subscribers
    rospy.Subscriber("/p3dx/odom", Odometry, robot_position_callback)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/task/commands", String, status_callback, queue_size=100)

    poses_pub = rospy.Publisher("/paths", PoseArray, queue_size=100)
    explore_pub = rospy.Publisher("/task/explore_commands", String, queue_size=100)
    explore_pub_info = rospy.Publisher("/task/explore_command_info", String, queue_size=100)

    # Loop rate
    rate = rospy.Rate(10)  # 10 Hz

    cell_size = 0.05  # meters per cell
    grid_width = 4000
    grid_height = 4000 

    while not rospy.is_shutdown():
        if grid is not None and robot_position is not None:
            global current_step, current_path

            if running:
                # Initial path calculation
                if current_path is None or new_path_needed == True:
                    new_path_needed = False
                    rospy.loginfo("Calculating new path")
                    start = world_to_grid(robot_position[0], robot_position[1], cell_size, grid_width, grid_height)
                    # end = goal

                    goal = None

                    explore_pub.publish("new_goal")
                    rospy.loginfo(rospy.Time.now())
                    goal = rospy.wait_for_message("/goals_new", PoseStamped)
                    rospy.loginfo("got goal")
                    explore_pub_info.publish("got")
                    if goal is not None:
                        gx = goal.pose.position.x
                        gy = goal.pose.position.y
                        end = (round((gx / 0.05) + 2000), round((gy / 0.05) + 2000))
                    else:
                        new_path_needed = True
                        continue
                    rospy.loginfo("Start: {}, End: {}".format(start, end))
                    new_path = astar_dynamic(grid, start, end)
                    rospy.loginfo(str(new_path))

                    if new_path:
                        new_path = smooth_current_path(new_path) # For smoothing
                        if False: # For visualising
                            pa = PoseArray()
                            poses = []
                            pa.header.frame_id = 'map'
                            s = ""
                            for n in new_path:
                                x = n[0]
                                y = n[1]
                                p = Pose()
                                if grid[x][y] != 0:
                                    s += "{}, {}, {} |".format(x, y, grid[x][y])
                                p.position.x = (x - 2000) * 0.05
                                p.position.y = (y - 2000) * 0.05
                                p.orientation.w = 1
                                poses.append(p)
                            pa.poses = poses
                            poses_pub.publish(pa)
                            rospy.loginfo(s)
                        current_path = new_path
                        current_step = 0  # Reset the current step

                # Process next step in the path
                if current_path and current_step < len(current_path) - 1:
                    rospy.loginfo("Processing next step in the current path")
                    next_cell = current_path[current_step + 1]

                    if not is_path_affected(current_path[current_step:], grid):
                        rospy.loginfo("Path is clear. Moving to next cell.")
                        detailed_movement_pub.publish(Point(x=next_cell[0], y=next_cell[1], z=0))
                        c_time = rospy.get_time()
                        time = rospy.wait_for_message("/robot_movement_completed", Float32)
                        while c_time > time.data:
                            time = rospy.wait_for_message("/robot_movement_completed", Float32)

                        # if is_close_to_target(next_cell):
                        current_step += 1
                        # else:
                        #     rospy.logwarn("Robot isn't close enough to estimated pose!")
                    else:
                        rospy.logwarn("Path is obstructed. Recalculating affected segment.")
                        affected_start, affected_end = find_affected_segment(current_path, grid)

                        if affected_start is not None and affected_start <= current_step:
                            affected_segment_start = current_path[affected_start]
                            affected_segment_end = current_path[affected_end]

                            new_path_segment = astar_dynamic(grid, affected_segment_start, affected_segment_end)

                            if new_path_segment:
                                current_path = current_path[:affected_start] + new_path_segment + current_path[affected_end + 1:]
                                publish_path(current_path, path_publisher, current_step)

                                # Now check if robot has moved to the next cell before incrementing current_step
                                if current_step < len(current_path) - 1:
                                    next_cell = current_path[current_step + 1]
                                    detailed_movement_pub.publish(Point(x=next_cell[0], y=next_cell[1], z=0))
                                    c_time = rospy.get_time()
                                    time = rospy.wait_for_message("/robot_movement_completed", Float32)
                                    while c_time > time.data:
                                        time = rospy.wait_for_message("/robot_movement_completed", Float32)
                                    # if is_close_to_target(next_cell):
                                    current_step += 1
                                    # else:
                                    #     rospy.logwarn("Robot isn't close enough to estimated pose!")
                elif current_path and current_step >= len(current_path) - 1:
                    # robot has got to position
                    current_path = None
                    new_path_needed = True

        rate.sleep()

    rospy.loginfo("Shutting down pathfinding node")

# Entry point for the program
if __name__ == '__main__':
    try:
        pathfinding_node()
    except rospy.ROSInterruptException:
        pass

