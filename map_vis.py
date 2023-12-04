#!/usr/bin/env python3

import rospy
import numpy as np
import pickle 
import base64
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

rospy.init_node("map_vis_node")

map_pub = rospy.Publisher("/map_vis", OccupancyGrid, queue_size=100)

while not rospy.is_shutdown():
    map_msg: String = rospy.wait_for_message("/grid_update", String)
    grid_map = pickle.loads(base64.b64decode(map_msg.data))
    blank_map = np.array([-1 for i in range(0, 4000*4000)]).reshape((4000, 4000))
    for gx, gy in grid_map:
        blank_map[gy][gx] = 100

    grid_msg = OccupancyGrid()
    origin = Pose()
    origin.position.x = -100
    origin.position.y = -100
    origin.orientation.w = 0
    grid_msg.info.origin = origin
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = "map"
    grid_msg.info.resolution = 0.05
    grid_msg.info.width = 4000
    grid_msg.info.height = 4000
    grid_msg.data = blank_map.ravel().tolist()
    map_pub.publish(grid_msg)