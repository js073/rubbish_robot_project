import numpy as np


def detect_static_obstacles(lidar_data, map_data, threshold):
    """
    Detect static obstacles susing LIDAR data and the known map.
    :param lidar_data: Processed LIDAR data in Cartesian coordinates.
    :param map_data: The map data from the SLAM system.
    :param threshold: Distance threshold to consider a point as an obstacle.
    :return: List or array of static obstacle coordinates.
    """
    static_obstacles = []
    for point in lidar_data:
        if not map_data.is_free_space(point) and map_data.distance_to_nearest_obstacle(point) > threshold:
            static_obstacles.append(point)
    return np.array(static_obstacles)


def detect_dynamic_obstacles(current_scan, previous_scan, distance_threshold, movement_threshold):
    """
    Detect dynamic obstacles by comparing consecutive LIDAR scans.
    :param current_scan: Current processed LIDAR scan data.
    :param previous_scan: Previous processed LIDAR scan data.
    :param distance_threshold: Threshold for considering a point as an obstacle.
    :param movement_threshold: Minimum movement required to consider an obstacle dynamic.
    :return: List or array of dynamic obstacle coordinates.
    """
    dynamic_obstacles = []
    for current_point, previous_point in zip(current_scan, previous_scan):
        if np.linalg.norm(current_point - previous_point) > movement_threshold and \
                np.linalg.norm(current_point) < distance_threshold:
            dynamic_obstacles.append(current_point)
    return np.array(dynamic_obstacles)


def detect_obstacles(current_lidar_data, previous_lidar_data, map_data):
    """
    Detect obstacles (both static and dynamic).
    :param current_lidar_data: Current processed LIDAR data.
    :param previous_lidar_data: Previous processed LIDAR data.
    :param map_data: The map data from the SLAM system.
    :return: Array containing obstacle coordinates.
    """
    static_obstacles = detect_static_obstacles(current_lidar_data, map_data, threshold=0.5)
    dynamic_obstacles = detect_dynamic_obstacles(current_lidar_data, previous_lidar_data, distance_threshold=20.0, movement_threshold=0.5)

    # Combine static and dynamic obstacles into a single set
    all_obstacles = set(static_obstacles) | set(dynamic_obstacles)
    return np.array(list(all_obstacles))


# Example usage (assuming you have the necessary LIDAR data and map data)
# current_lidar_data = # Current processed LIDAR data
# previous_lidar_data = # Previous processed LIDAR data
# map_data = # Map data from SLAM system
# static_obstacles, dynamic_obstacles = detect_obstacles(current_lidar_data, previous_lidar_data, map_data)
