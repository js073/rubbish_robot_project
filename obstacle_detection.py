#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from obstacle_detection_helpers import detect_obstacles, update_map_with_obstacles, process_lidar_data

class ObstacleDetectionNode:
    def __init__(self):
        rospy.init_node('obstacle_detection_node', anonymous=True)
        self.lidar_sub = rospy.Subscriber('/p3dx/laser/scan', LaserScan, self.lidar_callback)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.grid_pub = rospy.Publisher('/grid_update', OccupancyGrid, queue_size=10)
        self.robot_pose = None
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.static_obstacles = []
        
    def map_callback(self, map_msg):
        rospy.loginfo("Map callback triggered")
        grid_height = map_msg.info.height
        grid_width = map_msg.info.width

        # Create a grid of dimensions [grid_width x grid_height]
        temp = [[0 for _ in range(grid_height)] for _ in range(grid_width)]

        # Fill the grid with map data
        for i in range(len(map_msg.data)):
            grid_x = i % grid_width
            grid_y = i // grid_width
            temp[grid_x][grid_y] = map_msg.data[i]

        self.static_obstacles = self.process_map_data(temp)

    def process_map_data(self, map_data):
        static_obstacles = []
        for x in range(len(map_data)):
            for y in range(len(map_data[0])):
                if map_data[x][y] == 100:  # Occupied cell
                    static_obstacles.append((x, y))
        rospy.loginfo(f"Number of static obstacles detected: {len(static_obstacles)}")
        return list(set(static_obstacles))
 

    

    def pose_callback(self, data):
        rospy.loginfo("pose callback triggered")
        self.robot_pose = data.pose.pose
        
        position = self.robot_pose.position
        orientation = self.robot_pose.orientation
        rospy.loginfo(f"Robot Pose - Position: x={position.x}, y={position.y}, z={position.z}; "
                  f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

    def lidar_callback(self, data):
        rospy.loginfo("lidar callback triggered")
        if self.robot_pose is None:
            rospy.logwarn("Skipping LIDAR processing: Missing pose data")
            return

        angles = np.linspace(data.angle_min, data.angle_max, len(data.ranges))
        distances = np.array(data.ranges)
        lidar_data = np.column_stack((angles, distances))
        rospy.loginfo("Raw LIDAR data sample: %s", str(lidar_data[:5]))
        processed_data = process_lidar_data(lidar_data)
        rospy.loginfo("Processed LIDAR data sample: %s", str(processed_data[:5]))

        obstacles = detect_obstacles(self.robot_pose, processed_data, grid_resolution = grid_resolution)
        rospy.loginfo("Detected obstacles: %s", str(obstacles))
        
        rospy.loginfo("First 20 static obstacles: %s", str(self.static_obstacles[:20]))

        # Creating a grid map and update it with detected obstacles
        grid_map = update_map_with_obstacles(obstacles, self.static_obstacles , grid_size=(4000, 4000))
        
        # Publishing the updated grid map
        self.publish_grid(grid_map)

    def publish_grid(self, grid_map):
        rospy.loginfo("Publishing grid map")
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = grid_resolution
        grid_msg.info.width = grid_map.shape[1]
        grid_msg.info.height = grid_map.shape[0]
        grid_msg.data = grid_map.ravel().tolist()
        self.grid_pub.publish(grid_msg)

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting obstacle detection node")
        grid_resolution = 0.05
        node = ObstacleDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
