#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from nav_msgs.msg import OccupancyGrid
from obstacle_detection_helpers import detect_obstacles, update_map_with_obstacles, process_lidar_data
from std_msgs.msg import String, Byte
import map_inflation
import random
import pickle
import base64

class ObstacleDetectionNode:
    def __init__(self):
        rospy.init_node('obstacle_detection_node', anonymous=True)
        self.lidar_sub = rospy.Subscriber('/p3dx/laser/scan', LaserScan, self.lidar_callback)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.grid_pub = rospy.Publisher('/grid_update', String, queue_size=10, latch=True)
        self.robot_pose = None
        self.prev = None
        # self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.static_obstacles = []
        self.laser_scan = None
        self.map = None
        self.require_update = True
        self.prev_time = 0

    def map_callback(self, map_msg):
        rospy.loginfo("Map callback triggered")
        self.map = self.process_map_data(map_msg, map_msg.info.width, map_msg.info.height)
        rospy.loginfo("Processed map")

    def process_map_data(self, map_data, width, height):
        return np.array(map_inflation.inflate_map(map_data))

    def pose_callback(self, data: PoseWithCovarianceStamped):
        # rospy.loginfo("pose callback triggered")
        self.robot_pose = data.pose.pose
        dt = data.header.stamp
        ct = rospy.get_rostime()
        if dt.secs == ct.secs and (abs(ct.nsecs - dt.nsecs) / 1000000) <= 100:
            self.require_update = True

        # if self.map is not None:
        #     laser_scan = rospy.wait_for_message("/p3dx/laser/scan", LaserScan)
        #     self.lidar_callback(laser_scan)


        
    def lidar_callback(self, data: LaserScan):
        # rospy.loginfo("lidar callback triggered")
        self.laser_scan = data
        cp = self.robot_pose
        if self.robot_pose is None:
            rospy.logwarn("Skipping LIDAR processing: Missing pose or static map data")
            return

        dt = data.header.stamp
        ct = rospy.get_rostime()
        if dt.secs == ct.secs and (abs(ct.nsecs - dt.nsecs) / 1000000) <= 100 and self.require_update:
            # rospy.loginfo(abs(self.prev_time - ct) / 1000000)
            self.require_update = False
            rospy.loginfo("updating obstacles")
            # angles = np.linspace(data.angle_min, data.angle_max, len(data.ranges))
            distances = np.array(data.ranges)
            # lidar_data = np.column_stack((angles, distances))

            
            processed_data = process_lidar_data(distances, data.angle_min, data.angle_max)
            t = np.array(data.ranges)
            obstacles = detect_obstacles(cp, processed_data, grid_resolution=grid_resolution)
            # rospy.loginfo("Detected obstacles: %s", str(obstacles[:20]))

            # rospy.loginfo("First 20 static obstacles: %s", str(self.static_obstacles[:20]))
            current_map = np.array([])
            grid_map = update_map_with_obstacles(obstacles, current_map, grid_size=(4000, 4000))
            self.publish_grid(grid_map)
            rospy.loginfo("finished update")

    def publish_grid(self, grid_map):
        str_msg = String()
        ps = pickle.dumps(grid_map)
        str_msg.data = base64.b64encode(ps).decode('ascii')
        self.grid_pub.publish(str_msg)
        return
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
    # while not rospy.is_shutdown():
    #     rospy.loginfo("looping")
    #     pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
    #     node.pose_callback(pose)
