#!/usr/bin/env python3

import rospy 
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import MarkerArray, Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
import roslaunch
import subprocess
import os

class slam_mapping: 
    def __init__(self) -> None:
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.explore = subprocess.Popen(["roslaunch", "/opt/ros/noetic/share/explore_lite/launch/explore.launch"], stdout=subprocess.DEVNULL)
        self.frontiers = rospy.Subscriber("explore/frontiers", MarkerArray, self.frontier_callback, queue_size=100)
        self.info_pub = rospy.Publisher("control/info", String, queue_size=100)
        self.map_send = rospy.Publisher("/main/map_info", String, queue_size=100)
        self.map_rec = rospy.Subscriber("main/map_cmd", String, self.map_cmd_callback, queue_size=100)
        self.name = rospy.get_param('/auto_map/map_name')
        self.is_exploring = True
        self.explore_initialised = False
        self.map_saved = False
        self.returning_to_start = False

        print("started")
    
    def frontier_callback(self, msg):
        markers = msg.markers
        # print(markers[0])
        print(len(markers))
        if not(self.explore_initialised) and len(markers) > 0:
            self.explore_initialised = True
            print("here")
        if self.explore_initialised:
            no_ops = True
            for m in markers:
                if len(m.points) > 0: # If the current marker is a line
                    if m.color.r != 1: # Check if it is red
                        no_ops = False 
            if no_ops and not(self.map_saved): # Terminates the process if all of the frontiers are not accessible
                self.save_and_kill()

    def save_and_kill(self):
        self.end_explore() # kill explore
        # return to start pos 
        os.system(("rosrun map_server map_saver -f ~/maps/%s" % self.name))
        self.info_pub.publish("Saved and finished")
        self.map_saved = True

    def start_explore(self):
        self.explore = subprocess.Popen(["roslaunch", "/opt/ros/noetic/share/explore_lite/launch/explore.launch"], stdout=subprocess.DEVNULL)

    def end_explore(self):
        self.explore.terminate()

    def map_cmd_callback(self, msg):
        if msg.data == "shutdown":
            self.save_and_kill()
            client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            client.wait_for_server()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 0
            goal.target_pose.pose.position.y = 0
            goal.target_pose.pose.orientation = self.euler_to_quaternion(0, 0, 0)

            client.send_goal(goal)
            self.info_pub.publish("sent goal")
            wait = client.wait_for_result()
            self.info_pub.publish("got to goal")
            if client.get_result():
                self.map_send.publish("finished")
                self.info_pub.publish("good")
            else:
                self.map_send.publish("error")
                self.info_pub.publish("bad")
            # RETURN TO THE ORIGIN POS

    def euler_to_quaternion(self, yaw, pitch, roll):
        q = Quaternion()
        q.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        q.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        q.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return q

if __name__ == "__main__":
    rospy.init_node("slam_mapping")
    try:
        a = slam_mapping()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("hello")
