# Converts a normal occupancy grid map into one where the cells are the size of the robot

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from itertools import chain
from PIL import Image as im
import numpy as np

class mapping_convert:
    def __init__(self) -> None:
        self.map_rec = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=100)
        self.map_send = rospy.Publisher("/new_map", OccupancyGrid, queue_size=100)

    def map_callback(self, msg: OccupancyGrid):
        new_map:OccupancyGrid = convert_map(msg, 10)
        # print("map converted")
        new_map.info.map_load_time = msg.info.map_load_time
        # print(new_map)
        # self.map_send.publish(new_map)

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

def convert_2d_to_occupancy(td_array: [[int]], res: float):
    new_map = OccupancyGrid()
    new_map.info.resolution = res
    new_map.info.width = len(td_array)
    new_map.info.height = len(td_array)
    new_map.data = list(chain.from_iterable(td_array))
    return new_map

# Takes a map and the max dimension of the robot in meters
def convert_map(map: OccupancyGrid, reduction_factor: float):
    width = map.info.width

    while width % reduction_factor != 0: 
        reduction_factor += 1

    td_map = convert_occupancy_to_2d(map)

    new_map = reduce_map(td_map, reduction_factor)

    nm = np.array(new_map)
    nm = np.reshape(nm, (400, 400))
    img = im.fromarray(np.uint8(nm), 'L')
    # img.save("/home/jack/test.jpeg")

    return convert_2d_to_occupancy(new_map, map.info.resolution * reduction_factor)

# Reduces a given 2d array into new cells wit ha resolution of the old ones
def reduce_map(td_map: [[int]], cell_size: int):
    iters = int(len(td_map) / cell_size)
    new_map = []
    for i in range(0, iters):
        new_row = []
        for j in range(0, iters):
            ni = i * cell_size
            nj = j * cell_size
            current_cells = []
            for a in range(0, cell_size):
                for b in range(0, cell_size):
                    current_cells.append(td_map[ni+a][nj+b])
            
            new_row.append(determine_cell_value(current_cells))
        new_map.append(new_row)
    return new_map

def determine_cell_value(cells: [int]):
    if 100 in cells:
        return 100
    elif -1 in cells: 
        return 0
    else:
        return 50
    
if __name__ == "__main__":
    rospy.init_node("test")
    mapping_convert()
    rospy.spin()