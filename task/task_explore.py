#!/usr/bin/env python3
import rospy 
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
import math
import numpy as np
import map_inflation
import PIL.Image as im

SCAN_TOPIC = "/p3dx/laser/scan"
MAP_TOPIC = "/map"
POSE_TOPIC = "/amcl_pose"
GOAL_SEND_TOPIC = "/goals_new"
GOAL_FEEDBACK_TOPIC = "/main/commands"

VISITED_VALUE = 200 # The value that will be used to show the robot has observed a position
CLEAR_VALUE = 0
CAMERA_FOV = 70 # Field of view of the camera in degrees
FREE_AREA_SIZE = 5

class task_explore:
    def __init__(self) -> None:
        self.internal_map: [[int]] = None
        self.current_pose: Pose = None
        self.current_scan: LaserScan = None
        self.map_subscriber = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self.map_callback, queue_size=100)
        self.laser_subscriber = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.laser_callback, queue_size=100)
        self.pose_subsrciber = rospy.Subscriber(POSE_TOPIC, PoseWithCovarianceStamped, self.pose_callback, queue_size=100)
        self.goal_send = rospy.Publisher(GOAL_SEND_TOPIC, PoseStamped, queue_size=100)
        self.goal_feedback = rospy.Subscriber(GOAL_FEEDBACK_TOPIC, String, self.feedback_callback, queue_size=100)
        self.status_subscriber = rospy.Subscriber("/task/commands", String, self.command_callback, queue_size=100)
        self.map_set = False
        self.goal_set = False
        self.found_point = False
        self.end = False
        self.new_scan_needed = True
        self.map_pose = None
        self.current_heading = None
        self.found_point_vals = [-1, -1]
        rospy.loginfo("initialised")
        self.a = 1

    def map_callback(self, msg: OccupancyGrid):
        if not(self.map_set):
            self.internal_map = map_inflation.inflate_map(msg, 6)
            # self.internal_map = np.reshape(msg.data, (4000, 4000))
            self.map_set = True
            rospy.loginfo("map set")

    def laser_callback(self, msg: LaserScan):
        if self.new_scan_needed:
            self.current_scan = msg
            self.new_scan_needed = False
        pass

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.new_scan_needed = True
        rospy.loginfo("pose callback")
        while self.internal_map == None:
            rospy.sleep(1)
        self.current_pose = msg.pose.pose
        scan = self.current_scan
        map_pose = self.convert_pose_to_map(self.current_pose)
        current_heading = self.getHeading(self.current_pose.orientation)
        self.map_pose = map_pose
        self.current_heading = current_heading
        if scan != None:
            rospy.loginfo("ranges")
            ranges = self.get_used_ranges(scan)
            for r in ranges:
                if scan.range_min <= r[0] <= scan.range_max + 0.5:
                    self.change_map(r, current_heading, map_pose[0], map_pose[1])

    def command_callback(self, msg: String):
        s = msg.data
        if "pause" in s:
            pass
        elif "remsume" in s:
            pass

    def feedback_callback(self, msg: String):
        rospy.loginfo(msg.data)
        if 'run' in msg.data:
            self.goal_set = False
            if not(self.goal_set) and self.map_pose != None and self.current_heading != None:
                rospy.loginfo("goal")
                self.determine_goal(self.map_pose, self.current_heading)
        elif 'img' in msg.data:
            img = im.fromarray(np.uint8(self.internal_map), 'L')
            img.save("/home/jack/test_map%d.jpeg" % 50)
        return
        # implement depending on message type
        self.goal_set = False
        if self.end:
            # kill the node
            pass
        else:
            pose = [self.current_pose.position.x, self.current_pose.position.y]
            heading = self.getHeading(self.current_pose.orientation)
            self.determine_goal(pose, heading)

    def convert_pose_to_map(self, p: Pose): # Converts a pose to the equivalent grid cells, return coordinates (x, y)
        return [2000 + int(p.position.x / 0.05), 2000 + int(p.position.y / 0.05)]
    
    def convert_map_to_pose(self, x, y):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()
        p.header.seq = self.a
        self.a += 1
        p.pose.position.x = (x - 2000) * 0.05
        p.pose.position.y = (y - 2000) * 0.05
        p.pose.orientation = self.euler_to_quaternion(0, 0, 0)
        return p

    def get_used_ranges(self, scan: LaserScan):
        increment = scan.angle_increment
        fov = (CAMERA_FOV / 180) * math.pi
        number_increments = int((fov / increment) / 2)
        scans = scan.ranges
        mid_point = int(len(scans) / 2)
        new_scans = scans[(mid_point - number_increments):(mid_point + number_increments)]
        s = "min", min(new_scans)
        rospy.loginfo(s)

        new_scans = [[(s + 0.3 if s != math.inf and s!= 0 else 3.9), i*increment - (number_increments*increment)] for (i, s) in enumerate(new_scans)]
        return new_scans


    def getHeading(self, q: Quaternion):
        yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                        q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
        return yaw
    
    def change_map(self, range, heading, origin_x, origin_y):
        current_heading = heading + range[1]
        gradient = abs(math.tan(current_heading))
        quadrent = 0
        if 0 <= current_heading < math.pi / 2:
            quadrent = 1
        elif math.pi / 2 < current_heading <= math.pi:
            quadrent = 2
        elif 0 <= current_heading % math.pi < math.pi / 2:
            quadrent = 3
        elif math.pi / 2 < current_heading % math.pi < math.pi:
            quadrent = 4
        y = 0
        x = 0
        current_x = origin_x
        current_y = origin_y

        if quadrent != 0:
            while (math.sqrt(x**2 + y**2) * 0.05 <= min(range[0], 2)): 
                current_x = origin_x + x
                current_y = origin_y + y
                self.internal_map[current_y][current_x] = VISITED_VALUE
                x += 1 if quadrent == 1 or quadrent == 4 else -1
                y = round(gradient * abs(x)) * (1 if quadrent <= 2 else -1)

    def raytrace_viable_spaces(self, pos, heading):
        gradient = abs(math.tan(heading))
        rospy.loginfo(gradient)
        quadrent = 0
        if 0 <= heading < math.pi / 2:
            quadrent = 1
        elif math.pi / 2 < heading <= math.pi:
            quadrent = 2
        elif 0 <= heading % math.pi < math.pi / 2:
            quadrent = 3
        elif math.pi / 2 < heading % math.pi < math.pi:
            quadrent = 4

        rospy.loginfo(quadrent)
        count = 0
        map = self.internal_map
        ix = 0
        iy = 0
        if quadrent != 0:
            x = pos[0]
            y = pos[1]
            cx = 0
            cy = 0
            ix = x + cx
            iy = y + cy
            is_possible = True
            current_cell = map[y][x]
            while (current_cell == CLEAR_VALUE or current_cell == VISITED_VALUE) and 0 <= ix < 4000 and 0 <= iy < 4000:
                count += 1
                is_possible = self.is_viable_space(ix, iy)
                if is_possible:
                    # send the goal
                    s = 'found, x', ix, ',y', iy, 'its infront'
                    rospy.loginfo(s)
                    self.goal_set = True
                    return [ix, iy]
                else:
                    cx += 1 if quadrent == 1 or quadrent == 4 else -1
                    cy = (round(gradient * abs(cx)) * (1 if quadrent <= 2 else -1))
                    ix = x + cx
                    iy = y + cy
                if 0 <= ix < 4000 and 0 <= iy < 4000:
                    current_cell = map[iy][ix]
                else:
                    break
        s = 'x', ix, ',y', iy
        rospy.loginfo(s)
        return None
    

    def send_new_goal(self, x, y): # send the goal
        self.goal_set = True
        self.goal_send.publish(self.convert_map_to_pose(x, y))
        rospy.loginfo("set new goal")

    def determine_goal(self, pos, heading):
        rospy.loginfo("start rt")
        VARIANCE_ANGLE = math.pi / 4
        current = self.raytrace_viable_spaces(pos, heading)
        if current != None:
            self.send_new_goal(current[0], current[1])
            return
        current = self.raytrace_viable_spaces(pos, heading - VARIANCE_ANGLE)
        if current != None:
            self.send_new_goal(current[0], current[1])
            return
        current = self.raytrace_viable_spaces(pos, heading + VARIANCE_ANGLE)
        if current != None:
            self.send_new_goal(current[0], current[1])
            return
        self.found_point = False
        rospy.loginfo("end rt")
        position = self.get_nearest_space(pos[0], pos[1])
        if position != None:
            self.send_new_goal(position[0], position[1])
            # send new goal 
            pass
        else: 
            self.send_new_goal(2000, 2000) # Return to the starting position
            rospy.loginfo("not found")
            pass

    def get_nearest_space(self, x, y):
        map = self.internal_map
        distance_map = [] 
        i = 0 
        while i < 4000:
            j = 0 
            while j < 4000:
                current = map[i][j]
                if current == CLEAR_VALUE:
                    if self.is_viable_space(j, i):
                        distance_map.append([int(math.sqrt((j-x)**2 + (i-y)**2) / FREE_AREA_SIZE), j, i])
                j += FREE_AREA_SIZE
            i += FREE_AREA_SIZE
        # Could sort by smallest angle first 
        distance_map.sort(key=(lambda k: k[0]))
        if len(distance_map) == 0:
            return None
        d = distance_map[0]
        return [d[1], d[2]]

    
    def is_viable_space(self, x, y):
        if self.map_set:
            map = self.internal_map
            if map[y][x] == CLEAR_VALUE:
                mininum = - math.floor(FREE_AREA_SIZE / 2)
                maxinum = math.ceil(FREE_AREA_SIZE / 2)
                for yi in range (mininum, maxinum):
                    for xi in range(mininum, maxinum):
                        cx = x + xi
                        cy = y + yi 
                        if 0 <= cx < 4000 and 0 <= cy < 4000:
                            if map[cy][cx] != CLEAR_VALUE:
                                return False
                        else:
                            return False
                return True
        return False
    
    def euler_to_quaternion(self, yaw, pitch, roll):
        q = Quaternion()
        q.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        q.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        q.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return q


if __name__ == "__main__":
    rospy.init_node("task_explore")
    task_explore()
    rospy.spin()