#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
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
        self.static_obstacles = self.process_map_data(map_msg.data, map_msg.info.width, map_msg.info.height)

    def process_map_data(self, map_data, width, height):
        static_obstacles = []
        for i in range(len(map_data)):
            grid_x = i % width
            grid_y = i // width
            if map_data[i] == 100:  # Occupied cell
                # static_obstacles.append((grid_x, grid_y))
                static_obstacles.append((grid_x, grid_y))
        rospy.loginfo(f"Number of static obstacles detected: {len(static_obstacles)}")
        return list(set(static_obstacles))

    def pose_callback(self, data):
        rospy.loginfo("pose callback triggered")
        self.robot_pose = data.pose.pose
        
    def lidar_callback(self, data):
        rospy.loginfo("lidar callback triggered")
        if self.robot_pose is None or not self.static_obstacles:
            rospy.logwarn("Skipping LIDAR processing: Missing pose or static map data")
            return

        angles = np.linspace(data.angle_min, data.angle_max, len(data.ranges))
        distances = np.array(data.ranges)
        lidar_data = np.column_stack((angles, distances))
        processed_data = process_lidar_data(lidar_data, data.angle_min, data.angle_max)

        obstacles = detect_obstacles(self.robot_pose, processed_data, grid_resolution=grid_resolution)
        rospy.loginfo("Detected obstacles: %s", str(obstacles[:20]))

        rospy.loginfo("First 20 static obstacles: %s", str(self.static_obstacles[:20]))

        grid_map = update_map_with_obstacles(obstacles, self.static_obstacles, grid_size=(4000, 4000))
        self.publish_grid(grid_map)

    def publish_grid(self, grid_map):
        grid_msg = OccupancyGrid()
        origin = Pose()
        origin.position.x = -100
        origin.position.y = -100
        origin.orientation.w = 0
        grid_msg.info.origin = origin
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = grid_resolution
        grid_msg.info.width = grid_map.shape[1]
        grid_msg.info.height = grid_map.shape[0]
        grid_msg.data = grid_map.ravel().tolist()
        self.grid_pub.publish(grid_msg)


if __name__ == '__main__':
    grid_resolution = 0.05
    node = ObstacleDetectionNode()
    rospy.spin()
