import numpy as np
# Assuming you have these modules
from lidar_preprocessing import preprocess_lidar_data
from obstacle_detection import detect_obstacles
from ros_lidar_fetcher import LidarDataFetcher

# This function might be part of another module or the main script
# Global instance of LidarDataFetcher
lidar_fetcher = LidarDataFetcher()


def fetch_raw_lidar_data():
    """
    Fetch the latest raw LIDAR data from the ROS topic.
    :return: Raw LIDAR data.
    """
    return lidar_fetcher.fetch_raw_lidar_data()


def update_callback(map_data):
    """
    This function updates the map/grid based on the latest obstacle information.
    :param map_data: The map data from the SLAM system.
    :return: Updated map/grid
    """
    # Fetch the latest raw LIDAR data
    raw_lidar_data = fetch_raw_lidar_data()

    # Process the LIDAR data
    processed_data = preprocess_lidar_data(raw_lidar_data)

    # Detect obstacles
    obstacles = detect_obstacles(processed_data, processed_data, map_data)

    # Update the map/grid based on detected obstacles
    updated_map = update_map_with_obstacles(map_data, obstacles)

    return updated_map


def update_map_with_obstacles(map_data, obstacles):
    """
    Update the map/grid with new obstacle information.
    :param map_data: The current map data (2D array or similar structure).
    :param obstacles: List or array of obstacle coordinates.
    :return: Updated map/grid.
    """
    # Iterate through the map and update cells
    for x in range(len(map_data)):
        for y in range(len(map_data[x])):
            if (x, y) in obstacles:
                map_data[x][y] = 1  # Mark cell as an obstacle
            else:
                # Reset cell to traversable if not in the new obstacle set
                if map_data[x][y] == 1:
                    map_data[x][y] = 0

    return map_data

