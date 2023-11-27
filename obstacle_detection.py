#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry  # Import Odometry message
from obstacle_detection_helpers import detect_obstacles, update_map_with_obstacles, process_lidar_data

class ObstacleDetectionNode:
    def __init__(self):
        rospy.init_node('obstacle_detection_node', anonymous=True)
        self.lidar_sub = rospy.Subscriber('/base_scan', LaserScan, self.lidar_callback)

        # Subscriber for robot's pose using AMCL
        self.pose_sub = rospy.Subscriber('/amcl_pose', Odometry, self.pose_callback)

        # Subscriber for the known map
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Publisher for the updated grid
        self.grid_pub = rospy.Publisher('/grid_update', OccupancyGrid, queue_size=10)

        # Initialize your map data here
        self.map_data = None  # Initialize to None

        # Initialize robot's pose
        self.robot_pose = None

    def initialize_map(self, known_map_data):
        if known_map_data is None:
            rospy.logwarn("No known map data available. Initializing with an empty map.")
            return np.zeros((1, 1), dtype=np.int8)  # Initialize with an empty map

        return known_map_data  # Use the known map data as the initial map

    def map_callback(self, map_msg):
        # Extract known map data from the message
        map_width = map_msg.info.width
        map_height = map_msg.info.height
        map_data = np.array(map_msg.data).reshape((map_height, map_width))

        # Initialize the local map with known obstacles
        self.map_data = self.initialize_map(map_data)

    def pose_callback(self, data):
        # Update robot's pose when a new pose message is received
        self.robot_pose = data.pose.pose

    def lidar_callback(self, data):
        if self.robot_pose is None or self.map_data is None:
            return  # Skip if we don't have the robot's pose or known map yet

        # Convert LaserScan data to a format suitable for preprocessing
        angles = np.linspace(data.angle_min, data.angle_max, len(data.ranges))
        distances = np.array(data.ranges)
        lidar_data = np.column_stack((angles, distances))

        # Process LIDAR data
        processed_data = process_lidar_data(lidar_data)

        # Detect obstacles using the helper functions
        obstacles = detect_obstacles(self.robot_pose, processed_data, self.map_data, grid_resolution)

        # Update the map/grid based on detected obstacles
        self.map_data = update_map_with_obstacles(self.map_data, obstacles)

        # Publish the updated map/grid
        self.publish_grid()

    def publish_grid(self):
        if self.map_data is None:
            return  # Skip publishing if the map is not initialized yet

        # Convert your map data to a ROS OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # Set grid dimensions and resolution
        grid_msg.info.width = self.map_data.shape[1]
        grid_msg.info.height = self.map_data.shape[0]
        grid_msg.info.resolution = grid_resolution  # Adjust as needed

        # Convert the 2D numpy array to a 1D list for the OccupancyGrid message
        grid_msg.data = self.map_data.ravel().tolist()

        # Publish the grid
        self.grid_pub.publish(grid_msg)

if __name__ == '__main__':
    try:
        grid_resolution = 0.05  # Define your grid resolution here
        node = ObstacleDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
