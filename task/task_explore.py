import rospy 
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion
import math
import numpy as np

SCAN_TOPIC = "/base_scan"
MAP_TOPIC = "/map"
POSE_TOPIC = "/amcl_pose"
GOAL_SEND_TOPIC = "/goals"
GOAL_FEEDBACK_TOPIC = "/goal_info"

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
        self.pose_subsrciber = rospy.Subscriber(POSE_TOPIC, PoseWithCovariance, self.pose_callback, queue_size=100)
        self.goal_send = rospy.Publisher(GOAL_SEND_TOPIC, type, queue_size=100)
        self.goal_feedback = rospy.Publisher(GOAL_FEEDBACK_TOPIC, type, self.feedback_callback, queue_size=100)
        self.map_set = False
        self.goal_set = False
        self.found_point = False
        self.end = False
        self.found_point_vals = [-1, -1]

    def map_callback(self, msg: OccupancyGrid):
        if not(self.map_set):
            self.internal_map = np.reshape(msg.data, (4000, 4000))
            self.map_set = True

    def laser_callback(self, msg: LaserScan):
        self.current_scan = msg
        pass

    def pose_callback(self, msg: PoseWithCovariance):
        self.current_pose = msg.pose
        scan = self.current_scan
        map_pose = self.convert_pose_to_map(self.current_pose)
        current_heading = self.getHeading(msg.pose.orientation)
        if scan != None:
            ranges = self.get_used_ranges(scan)
            for r in ranges:
                if scan.range_min <= r[0] <= scan.range_max:
                    self.change_map(r, current_heading, map_pose[0], map_pose[1])
        if not(self.goal_set):
            self.determine_goal(self, map_pose, current_heading)
            self.goal_set = True

    def feedback_callback(self, msg):
        # implement depending on message type
        self.goal_set = False
        if self.end:
            # kill the node
            pass
        else:
            pose = [self.current_pose.position.x, self.current_pose.position.y]
            heading = self.getHeading(self.current_pose.orientation)
            self.determine_goal(pose, heading)

    def convert_pose_to_map(self, pose: Pose): # Converts a pose to the equivalent grid cells, return coordinates (x, y)
        return [2000 + int(pose.position.x / 0.05), 2000 + int(pose.position.y / 0.05)]

    def get_used_ranges(self, scan: LaserScan):
        increment = scan.angle_increment
        fov = (CAMERA_FOV / 180) * math.pi
        number_increments = int((fov / increment) / 2)
        scans = scan.ranges
        mid_point = int(len(scans) / 2)
        new_scans = scans[(mid_point - number_increments):(mid_point + number_increments)]
        new_scans = [[s + 0.3, i*increment] for (i, s) in enumerate(new_scans)]
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
                self.internal_map[current_y][current_x] = 50
                x += 1 if quadrent == 1 or quadrent == 4 else -1
                y = round(gradient * abs(x)) * (1 if quadrent <= 2 else -1)

    def raytrace_viable_spaces(self, pos, heading):
        gradient = abs(math.tan(heading))
        print(gradient)
        quadrent = 0
        if 0 <= heading < math.pi / 2:
            quadrent = 1
        elif math.pi / 2 < heading <= math.pi:
            quadrent = 2
        elif 0 <= heading % math.pi < math.pi / 2:
            quadrent = 3
        elif math.pi / 2 < heading % math.pi < math.pi:
            quadrent = 4

        print(quadrent)
        count = 0
        map = self.internal_map
        if quadrent != 0:
            x = pos[0]
            y = pos[1]
            cx = 0
            cy = 0
            ix = x + cx
            iy = y + cy
            is_possible = True
            current_cell = map[y][x]
            while (current_cell == CLEAR_VALUE or current_cell == VISITED_VALUE or current_cell == 200) and 0 <= ix < 4000 and 0 <= iy < 4000:
                count += 1
                self.internal_map[iy][ix] = 200
                is_possible = self.is_viable_space(ix, iy)
                if is_possible:
                    # send the goal
                    print('found, x', ix, ',y', iy)
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
        print('x', ix, ',y', iy)
        return None


    def determine_goal(self, pos, heading):
        print("start rt")
        VARIANCE_ANGLE = math.pi / 4
        if self.raytrace_viable_spaces(pos, heading) != None:
            return
        if self.raytrace_viable_spaces(pos, heading - VARIANCE_ANGLE) != None:
            return
        if self.raytrace_viable_spaces(pos, heading + VARIANCE_ANGLE) != None:
            return
        self.found_point = False
        print("end rt")
        position = self.get_nearest_space(pos[0], pos[1])
        if position != None:
            self.goal_set = True
            print("dopne", position)
            # send new goal 
            pass
        else: 
            # return to start position
            print("not found") 
            self.goal_set = True
            pass

    def get_nearest_space(self, x, y):
        map = self.internal_map
        distance_map = [] 
        i = 0 
        while i < 4000:
            j = 0 
            while j < 4000:
                current = map[i][j]
                if current == CLEAR_VALUE or current == 200:
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
            if map[y][x] == CLEAR_VALUE or map[y][x] == 200:
                mininum = - math.floor(FREE_AREA_SIZE / 2)
                maxinum = math.ceil(FREE_AREA_SIZE / 2)
                for yi in range (mininum, maxinum):
                    for xi in range(mininum, maxinum):
                        cx = x + xi
                        cy = y + yi 
                        if 0 <= cx < 4000 and 0 <= cy < 4000:
                            if map[cy][cx] != CLEAR_VALUE and map[cy][cx] != 200:
                                return False
                        else:
                            return False
                return True
        return False


if __name__ == "__main__":
    rospy.init_node("task_explore")
    task_explore()
    rospy.spin()