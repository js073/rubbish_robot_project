#!/usr/bin/env python3

import rospy 
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import MarkerArray, Marker
import math
import roslaunch
import subprocess
import os

class slam_mapping: 
    def __init__(self) -> None:
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.explore = roslaunch.parent.ROSLaunchParent(uuid, ["/opt/ros/noetic/share/explore_lite/launch/explore.launch"])
        self.frontiers = rospy.Subscriber("explore/frontiers", MarkerArray, self.frontier_callback, queue_size=100)
        self.info_pub = rospy.Publisher("control/info", String, queue_size=100)
        self.name = "test1"
        self.is_exploring = True
        self.explore_initialised = False
        self.map_saved = False
    
    def frontier_callback(self, msg):
        rospy.logdebug("HEREH\n\n\n\n\n\n\n\n")
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
                self.end_explore() # kill explore
                print("killed")
                # return to start pos 
                os.system(("rosrun map_server map_saver -f ~/maps/%s" % self.name))
                self.info_pub.publish("Saved and finished")
                self.map_saved = True

    def start_explore(self):
        self.explore.start()

    def end_explore(self):
        self.explore.shutdown()

if __name__ == "__main__":
    rospy.init_node("slam_mapping")
    a = slam_mapping()
    a.start_explore()
    rospy.spin()
