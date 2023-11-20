#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan


class LidarDataFetcher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('lidar_data_fetcher', anonymous=True)

        # Subscribe to the LIDAR data topic
        # Replace 'lidar_topic' with the actual topic name
        rospy.Subscriber('lidar_topic', LaserScan, self.lidar_callback)

        # Variable to hold LIDAR data
        self.current_lidar_data = None

    def lidar_callback(self, data):
        # Callback function for processing LIDAR data
        self.current_lidar_data = data

    def fetch_raw_lidar_data(self):
        # Wait for the latest data
        rospy.wait_for_message('lidar_topic', LaserScan)
        return self.current_lidar_data


# Example usage
if __name__ == '__main__':
    fetcher = LidarDataFetcher()
    raw_lidar_data = fetcher.fetch_raw_lidar_data()
    # Now, raw_lidar_data contains the latest LIDAR data
