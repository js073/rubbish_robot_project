#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Point, PointStamped
from std_msgs.msg import String, Bool, Time, Float32
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
        self.info = rospy.Publisher("/robot_movement_completed", Float32, queue_size=100, latch=True)
        self.temp = rospy.Publisher("/stamped", PointStamped, queue_size=100)
        self.status_subscriber = rospy.Subscriber("/task/commands", String, self.command_callback, queue_size=100)
        self.is_goal = False
        self.goal_pose = None
        self.run = True
    

    def oc(self, msg: Odometry):
        if self.is_goal and self.run:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            # rospy.loginfo(self.goal_pose)
            delta_x = (self.goal_pose.x - p.x)
            delta_y = (self.goal_pose.y - p.y)
            theta = math.atan2(delta_y, delta_x)
            oe = self.getHeading(o)
            t = Twist()
            dist = math.sqrt((p.x - self.goal_pose.x)**2 + (p.y - self.goal_pose.y)**2)
            if dist <= 0.1:
                t
                rospy.loginfo("reached goal")
                time = rospy.get_time()
                f = Float32()
                f.data = time
                self.info.publish(f)
                self.is_goal = False
            elif abs(theta - oe) <= 0.1:
                t.linear.x = 0.5
                t.angular.z = 0
                # rospy.loginfo("moving forward")
                f = Float32()
                f.data = -1
                self.info.publish(f)
            else: 
                if self.left_or_right(oe, theta):
                    t.angular.z = 0.5
                else: 
                    t.angular.z = -.5
                f = Float32()
                f.data = -1
                self.info.publish(f)
            s = "linear: {}, angular: {}, l or r: {}".format(round(abs(oe - theta), 4), round(dist, 4), self.left_or_right(oe, theta))
            # rospy.loginfo(s)
            self.move.publish(t)
            # rospy.loginfo(t)

    def left_or_right(self, ca, ga):
        ca = (ca / math.pi) * 180
        ga = (ga / math.pi) * 180
        diff = ga - ca
        diff = (diff + 180) % 360 - 180
        # diff += -360 if diff > 180 else 360
        return diff > 0

    def command_callback(self, msg):
        if "pause" in msg.data:
            self.run = False
        elif "resume" in msg.data:
            self.run = True
            

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
    
    def inverse_quaterion(self, q: Quaternion):
        inv = Quaternion()
        inv.x = -q.x
        inv.y = -q.y
        inv.x = -q.x
        inv.w = q.w
        return inv

if __name__ == "__main__":
    rospy.init_node("movement")
    movement()
    rospy.spin()
