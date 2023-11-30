import rospy 
from nav_msgs.msg import OccupancyGrid, MapMetaData
import math
import numpy as np
import PIL.Image as im
import time
import matplotlib
import matplotlib.pyplot as plt

OBSTACLE_VALUE = 100 # Value that obstacles on the grid map are given
INFLATION_SIZE = 5 # Needs to be HALF the size of the robot

# Takes an OccupancyGrid and converts it into a 2D array
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

# Will take an occupancy grid and inlfate all obstacles by a factor of size
def inflate_map(map: OccupancyGrid, size=INFLATION_SIZE):
    map_2d = convert_occupancy_to_2d(map)
    rospy.loginfo("done conv")
    # map_2d_to_img(map_2d, "normal_map")
    rospy.loginfo("saved normal")
    obstacle_coordinates = get_obstacle_coordinates(map_2d)
    new_map = map_2d
    for (x, y) in obstacle_coordinates:
        new_map = inflate_point(new_map, size, x, y)
    rospy.loginfo("inflated map")
    # map_2d_to_img(new_map, "inflated_map") # Generate image (testing)
    rospy.loginfo("saved infl")
    return new_map

# Gets the coordinates of all map obstacles
def get_obstacle_coordinates(map_2d: [[int]]):
    points = []
    for (y, v) in enumerate(map_2d):
        for (x, v2) in enumerate(v):
            if v2 == OBSTACLE_VALUE:
                points.append((x, y))
    return points

# Inflates the points by the speicified size
def inflate_point(map_2d: [[int]], size: int, coord_x: int, coord_y: int):
    x_points = list(range(-size, size+1))
    y_points = list(range(-size, size+1))
    points = []

    for x in x_points:
        for y in y_points:        
            if math.floor(math.sqrt((x**2) + (y**2))) <= size:
                points.append((x + coord_x, y + coord_y))

    points = [(x, y) for (x, y) in points if x >= 0 and x < len(map_2d) and y >= 0 and y < len(map_2d)]
    
    for (x, y) in points:
        map_2d[y][x] = 100

    return map_2d

# Generates an image for testing purposes
def map_2d_to_img(map_2d: [[int]], filename):
    new = []
    for a in map_2d:
        for b in a:
            if b == 0:
                new.append(255)
            elif b == 100:
                new.append(0)
            else:
                new.append(150)
    nm = np.array(new)
    nm = np.reshape(nm, (4000, 4000))
    img = im.fromarray(np.uint8(nm), 'L')
    img.save("/home/jack/%s.jpeg" % filename)


# FOR TESTING
class mapping_inflation:
    def __init__(self) -> None:
        self.map_rec = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=100)
        self.map_send = rospy.Publisher("/new_map", OccupancyGrid, queue_size=100)

    def map_callback(self, msg: OccupancyGrid):
        times = []
        matplotlib.use('agg')
        for i in range(0, 100):
            t0 = time.process_time()
            inflate_map(msg)
            t1 = time.process_time()
            print("Time difference is %f seconds" % (t1-t0)) # Determines computation time
            times.append(t1 - t0)
        rospy.loginfo(sum(times) / len(times))
        plt.boxplot(times)
        plt.savefig("map_infl_data.jpg") 

if __name__ == "__main__" and True: # Set to true for testing 
    rospy.init_node("infl_map")
    mapping_inflation()
    rospy.spin()