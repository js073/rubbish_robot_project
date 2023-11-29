#!/usr/bin/env python3
import rospy 
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import math
from . import map_inflation

class exploration:
    def __init__(self):
        scan_topic_name = "base_scan"
        map_topic_name = "map"
        odometry_topic_name = "odom"
        movement_topic_name = "cmd_vel"
        goal_topic_name = "move_base_simple/goal"

        self.map = None
        self.goal_mode = False
        self.target_angle = 0
        self.target_coord = None
        self.previous_angle = 0


        self.scan_sub = rospy.Subscriber(scan_topic_name, LaserScan, self.scan_callback, queue_size=100)
        self.map_sub = rospy.Subscriber(map_topic_name, OccupancyGrid, self.map_callback, queue_size=100)
        self.movement_pub = rospy.Publisher(movement_topic_name, Twist, queue_size=100)
        self.odometry_sub = rospy.Subscriber(odometry_topic_name, Odometry, self.odom_callback, queue_size=100)
        self.goal_sub = rospy.Subscriber(goal_topic_name, PoseStamped, self.goal_callback, queue_size=100)

    def goal_callback(self, msg):
        goal_pose = msg.pose
        goal_pos = goal_pose.position
        self.goal_mode = True
        required_angle = math.atan2(goal_pos.y - self.estimated_pose.position.y, goal_pos.x - self.estimated_pose.position.x)
        # self.target_angle = required_angle
        self.target_coord = self.rotateQuaternion(self.estimated_pose.orientation, required_angle)

    def get_heading(self, q):
        yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
        return yaw
    
    def get_map_coord(self, pos: Point()) -> int:
        if self.map == None:
            return -100
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height
        

    def scan_callback(self, msg):
        return
    
    def map_callback(self, msg):
        self.map = map_inflation.inflate_map(msg, 6)
        # na = np.array(msg.data)
        # for u in np.unique(na):
        #     print(u, ":", len(list(filter(lambda x: x == u, msg.data))))
        # return
    
    def odom_callback(self, msg):
        estimated_pose = msg.pose.pose
        self.estimated_pose = estimated_pose
        cmd_vel = Twist()
        # if (self.goal_mode):
        #     if self.target_angle != self.estimated_pose.orientation:
        #         cmd_vel.angular.z = 1
        #         print("turning")
        #     else:
        #         cmd_vel.linear.x = 1
        tmp = self.get_heading(estimated_pose.orientation)
        print(self.previous_angle - tmp)
        self.previous_angle = tmp
        
        cmd_vel.angular.z = 0.5 # 1 here = ~0.1 radians
        self.movement_pub.publish(cmd_vel)
        # print(estimated_position.x, estimated_position.y, estimated_position.z)
    

if __name__ == "__main__":
    node_name = "robot_exploration"
    rospy.init_node(node_name)
    exploration()
    rospy.spin()