import rospy 
from nav_msgs.msg import OccupancyGrid

class test:
    def __init__(self) -> None:
        self.map_recieve = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=100)

    def map_callback(self, msg: OccupancyGrid):
        print(msg.info.origin)

if __name__ == "__main__":
    rospy.init_node("test_node")
    test()
    rospy.spin()