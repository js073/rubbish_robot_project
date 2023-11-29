#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Point, PointStamped
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

class movement: 
    def __init__(self) -> None:
        self.odom = rospy.Subscriber("/p3dx/odom", Odometry, self.oc, queue_size=100)
        self.goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.gc, queue_size=100)
        self.ng = rospy.Subscriber("/detailed_robot_movement_request", Point, self.ngc)
        self.move = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
        self.info = rospy.Publisher("/robot_movement_completed", Bool, queue_size=100)
        self.temp = rospy.Publisher("/stamped", PointStamped, queue_size=100)
        self.is_goal = False
        self.goal_pose = None
    

    def oc(self, msg: Odometry):
        if self.is_goal:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            rospy.loginfo(self.goal_pose)
            delta_x = (self.goal_pose.x - p.x)
            delta_y = (self.goal_pose.y - p.y)
            theta = math.atan2(delta_y, delta_x)
            oe = self.getHeading(o)
            t = Twist()
            dist = math.sqrt((p.x - self.goal_pose.x)**2 + (p.y - self.goal_pose.y)**2)
            if dist <= 0.05:
                t
                rospy.loginfo("reached goal")
                self.info.publish(True)
            elif abs(theta - oe) <= 0.1:
                t.linear.x = 0.5
                t.angular.z = 0
                rospy.loginfo("moving forward")
            else: 
                if oe < theta:
                    t.angular.z = 0.5
                else: 
                    t.angular.z = -.5
                rospy.loginfo(abs(oe -theta))
            self.move.publish(t)
            

    def ngc(self, msg: Point):
        a = msg
        a.x = round(msg.x - 2000) * 0.05
        a.y = round(msg.y - 2000) * 0.05
        self.goal_pose = a
        b = PointStamped()
        b.point = a
        b.header.frame_id = "/map"
        self.temp.publish(b)
        self.is_goal = True


    def gc(self, msg: PoseStamped):
        self.goal_pose = msg.pose.position
        self.is_goal = True

    def getHeading(self, q: Quaternion):
        yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
        return yaw

if __name__ == "__main__":
    rospy.init_node("movement")
    movement()
    rospy.spin()
