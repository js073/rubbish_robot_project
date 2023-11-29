#!/usr/bin/env python3

import rospy, subprocess
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import numpy as np

class test2:
    def __init__(self) -> None:
        self.amcl = subprocess.run(['rosrun', 'amcl', 'amcl', 'scan:=p3dx/laser/scan' ,'_use_map_topic:=true', '_odom_frame_id:=p3dx_tf/odom', '_base_frame_id:=p3dx_tf/base_link'])
        i_pose = PoseWithCovarianceStamped()
        i_pose.pose.pose.position.x = 5
        i_pose.pose.pose.position.y = 5
        i_pose.pose.pose.orientation = self.euler_to_quaternion(0, 0, 0)
        self.init_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=100)
        self.init_pose_pub.publish(i_pose)

    def euler_to_quaternion(self, yaw, pitch, roll):
        q = Quaternion()
        q.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        q.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        q.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return q
    
if __name__ == "__main__":
    rospy.init_node("test2node")
    test2()
    rospy.spin()