import rospy 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
import PIL.Image as im
import math

CLEAR_VALUE = 0
VISITED_VALUE = 50
FREE_AREA_SIZE = 5

def map_2d_to_img(map_2d: [[int]]):
    nm = np.array(map_2d)
    nm = np.reshape(nm, (4000, 4000))
    img = im.fromarray(np.uint8(nm), 'L')
    img.save("/home/jack/test_map%d.jpeg" % 1)
    print("saved")

def convert_occupancy_to_2d(map: OccupancyGrid):
    meta_data = map.info
    data = map.data

    width = meta_data.width
    height = meta_data.height

    td_array = []
    for i in range(0, height):
        sub = []
        for j in range(0, width):
            current_index = (i * width) + j
            sub.append(data[current_index])
        td_array.append(sub)

    return td_array

class test:
    def __init__(self) -> None:
        self.map_recieve = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=100)
        self.scan_rec = rospy.Subscriber('/base_scan', LaserScan, self.sc, queue_size=100)
        self.done = False
        self.internal_map = None
        self.map_set = False
        self.temp = None

    def map_callback(self, msg: OccupancyGrid):
        print(msg.info)
        nm = np.reshape(msg.data, (4000, 4000))
        v, c = np.unique(nm, return_counts=True)
        for i in range (0, len(v)):
            print(v, ":", c)
        self.internal_map = nm
        self.map_set = True
        return
        a = convert_occupancy_to_2d(msg)
        map_2d_to_img(a)

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
                    self.temp = [ix, iy]
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
        # VARIANCE_ANGLE = math.pi / 4
        # if self.raytrace_viable_spaces(pos, heading) != None:
        #     return
        # if self.raytrace_viable_spaces(pos, heading - VARIANCE_ANGLE) != None:
        #     return
        # if self.raytrace_viable_spaces(pos, heading + VARIANCE_ANGLE) != None:
        #     return
        self.temp = [0, 0]
        self.found_point = False
        print("end rt")
        position = self.get_nearest_space(pos[0], pos[1])
        if position != None:
            self.goal_set = True
            print("dopne", position)
            self.temp = position
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

    def sc(self, msg):
        if self.done == False and self.map_set:
            self.done = True
            print("start")
            ranges = self.get_used_ranges(msg)
            print(len(ranges))
            for r in ranges:
                self.change_map(r, 0, 2000, 2000)
            print("stop")
            self.determine_goal([2000, 2000], 0)
            a = self.temp
            mini = - math.floor(FREE_AREA_SIZE / 2)
            maxi = math.ceil(FREE_AREA_SIZE / 2)
            for i in range(mini, maxi):
                for j in range(mini, maxi):
                    self.internal_map[a[1]+i][a[0]+j] = -1
            print('goals')
            img = im.fromarray(np.uint8(self.internal_map), 'L')
            img.save("/home/jack/test_map%d.jpeg" % 50)
            print("save")
            

    def euler_to_quaternion(self, yaw, pitch, roll):
        q = Quaternion()
        q.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        q.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        q.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return q
    
    def get_used_ranges(self, scan: LaserScan):
        increment = scan.angle_increment
        fov = (90 / 180) * math.pi
        number_increments = int((fov / increment) / 2)
        scans = scan.ranges
        mid_point = int(len(scans) / 2)
        new_scans = scans[(mid_point - number_increments):(mid_point + number_increments)]
        new_scans = [[s + 0.3, (i*increment) - (number_increments*increment)] for (i, s) in enumerate(new_scans)]
        return new_scans

if __name__ == "__main__":
    rospy.init_node("test_node")
    t = test()
    rospy.spin()
    print(t.euler_to_quaternion(0, 0, 0))


