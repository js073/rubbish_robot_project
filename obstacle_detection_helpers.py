import numpy as np

def filter_noise(lidar_data):
    """
    Apply a median filter to reduce noise in LIDAR data.
    :param lidar_data: A numpy array of LIDAR measurements.
    :return: Filtered LIDAR data.
    """
    # Replace np.median with a different filter if needed
    filtered_data = np.median(lidar_data, axis=1)
    return filtered_data

def polar_to_cartesian(lidar_data):
    """
    Convert LIDAR data from polar to Cartesian coordinates.
    :param lidar_data: A numpy array of LIDAR measurements.
    :return: LIDAR data in Cartesian coordinates.
    """
    angles = lidar_data[:, 0]  # Angles are assumed to be in the first column
    distances = lidar_data[:, 1]  # Distances in the second column
    x = distances * np.cos(angles)
    y = distances * np.sin(angles)
    return np.column_stack((x, y))


def process_lidar_data(raw_lidar_data):
    """
    Preprocess raw LIDAR data.
    :param raw_lidar_data: A numpy array of raw LIDAR measurements.
    :return: Processed LIDAR data in Cartesian coordinates.
    """
    filtered_data = filter_noise(raw_lidar_data)
    cartesian_data = polar_to_cartesian(filtered_data)
    return cartesian_data

def convert_to_map_coordinates(robot_pose, point, grid_resolution):
    """
    Convert LIDAR point from polar to Cartesian coordinates and then to map grid coordinates.
    :param robot_pose: The pose of the robot in the map frame (x, y, theta).
    :param point: A tuple (angle, distance) representing the LIDAR point in polar coordinates.
    :param grid_resolution: The resolution of the grid map.
    :return: (grid_x, grid_y) coordinates in the map's grid.
    """
    angle, distance = point
    x = distance * np.cos(angle)
    y = distance * np.sin(angle)

    # Transform based on robot's pose
    map_x = robot_pose.x + x * np.cos(robot_pose.theta) - y * np.sin(robot_pose.theta)
    map_y = robot_pose.y + x * np.sin(robot_pose.theta) + y * np.cos(robot_pose.theta)

    # Convert to grid coordinates
    grid_x = int(map_x / grid_resolution)
    grid_y = int(map_y / grid_resolution)

    return grid_x, grid_y

def detect_obstacles(robot_pose, lidar_data, map_data, grid_resolution):
    """
    Detect obstacles using the current LIDAR data.
    :param robot_pose: The pose of the robot in the map frame (x, y, theta).
    :param lidar_data: Current processed LIDAR data.
    :param map_data: The map data (2D array).
    :param grid_resolution: The resolution of the grid map.
    :return: List of grid coordinates representing obstacles.
    """
    obstacles = []
    for point in lidar_data:
        grid_x, grid_y = convert_to_map_coordinates(robot_pose, point, grid_resolution)
        if 0 <= grid_x < len(map_data) and 0 <= grid_y < len(map_data[0]):
            if map_data[grid_x][grid_y] == 0:  # Check if the cell is not already marked as an obstacle
                obstacles.append((grid_x, grid_y))
    return obstacles

def update_map_with_obstacles(map_data, obstacles):
    """
    Update the map/grid with new obstacle information.
    :param map_data: The current map data (2D array).
    :param obstacles: List of grid coordinates representing obstacles.
    :return: Updated map/grid.
    """
    for x, y in obstacles:
        if 0 <= x < len(map_data) and 0 <= y < len(map_data[0]):
            map_data[x][y] = 1  # Mark cell as an obstacle
    return map_data
