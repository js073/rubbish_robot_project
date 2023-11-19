import rospy 
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import math

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
    
    def rotateQuaternion(self, q_orig, yaw):
        """
        Converts a basic rotation about the z-axis (in radians) into the
        Quaternion notation required by ROS transform and pose messages.
        
        :Args:
        | q_orig (geometry_msgs.msg.Quaternion): to be rotated
        | yaw (double): rotate by this amount in radians
        :Return:
        | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
        """
        # Create a temporary Quaternion to represent the change in heading
        q_headingChange = Quaternion()

        p = 0
        y = yaw / 2.0
        r = 0
    
        sinp = math.sin(p)
        siny = math.sin(y)
        sinr = math.sin(r)
        cosp = math.cos(p)
        cosy = math.cos(y)
        cosr = math.cos(r)
    
        q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
        q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
        q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
        q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

        # ----- Multiply new (heading-only) quaternion by the existing (pitch and bank) 
        # ----- quaternion. Order is important! Original orientation is the second 
        # ----- argument rotation which will be applied to the quaternion is the first 
        # ----- argument. 
        return self.multiply_quaternions(q_headingChange, q_orig)


    def multiply_quaternions(self, qa, qb ):
        """
        Multiplies two quaternions to give the rotation of qb by qa.
        
        :Args:
        | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
        | qb (geometry_msgs.msg.Quaternion): to rotate by qa
        :Return:
        | (geometry_msgs.msg.Quaternion): qb rotated by qa.
        """
        combined = Quaternion()
        
        combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
        combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
        combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
        combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
        return combined
    
    def get_map_coord(self, pos: Point()) -> int:
        if self.map == None:
            return -100
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height
        

    def scan_callback(self, msg):
        return
    
    def map_callback(self, msg):
        self.map = msg
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