#!/usr/bin/env python3
import rospy, rosnode
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import math

class task_controller:
    def __init__(self) -> None:
        # self.laser_sub = rospy.Subscriber('/p3dx/laser/scan', LaserScan, self.scan_callback, queue_size=100)
        self.pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=100)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback, queue_size=100)
        self.odom_sub = rospy.Subscriber("/p3dx/odom", Odometry, self.odom_callback, queue_size=100)
        self.p = rospy.Publisher('/inf', String, queue_size=100)
        self.pose_set = False
        self.pose_recieved = False
        self.count = 0
        self.pose = None

    def odom_callback(self, msg: Odometry):
        s = 's'
        if not(self.pose_set):
            s += " ps "
        if '/amcl' in rosnode.get_node_names():
            s += " nodes "
        if not(self.pose_recieved):
            s += " pr "
        # rospy.loginfo(s)
        if not(self.pose_set) and '/amcl' in rosnode.get_node_names() and not(self.pose_recieved):
            rospy.loginfo("called")
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            if self.pose != None:
                dist = math.sqrt((self.pose.x - pos.x)**2 + (self.pose.y - pos.y)**2)
                if dist > 0.3:
                    p = PoseWithCovarianceStamped()
                    p.header.stamp = rospy.Time().now()
                    p.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
                    p.header.frame_id = "map"
                    p.pose.pose.position.x = pos.x
                    p.pose.pose.position.y = pos.y
                    p.pose.pose.orientation = ori
                    self.pose_pub.publish(p)
                    rospy.loginfo("called")
                    self.pose_set = True

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.pose = msg.pose.pose.position
        rospy.loginfo("------------------ POSE SET")
        # self.pose_recieved = True


    def euler_to_quaternion(self, yaw, pitch, roll):
        q = Quaternion()
        q.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        q.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        q.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return q
    
if __name__ == "__main__":
    rospy.init_node("task_controller")
    task_controller()
    rospy.spin()