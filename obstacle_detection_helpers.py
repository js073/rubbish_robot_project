import numpy as np
import tf

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
    angles = np.linspace(-np.pi/2, np.pi/2, len(lidar_data))
    distances = lidar_data
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

def convert_to_map_coordinates(robot_pose, cartesian_point, grid_resolution):
    """
    Convert a point from Cartesian coordinates to map grid coordinates.
    :param robot_pose: The pose of the robot in the map frame.
    :param cartesian_point: A tuple (x, y) representing the point in Cartesian coordinates.
    :param grid_resolution: The resolution of the grid map.
    :return: (grid_x, grid_y) coordinates in the map's grid.
    """
    x, y = cartesian_point

    # Extract position
    pos_x = robot_pose.position.x
    pos_y = robot_pose.position.y

    # Convert quaternion to Euler angles (roll, pitch, yaw)
    quaternion = (
        robot_pose.orientation.x,
        robot_pose.orientation.y,
        robot_pose.orientation.z,
        robot_pose.orientation.w
    )
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

    # Transform based on robot's pose
    map_x = pos_x + x * np.cos(yaw) - y * np.sin(yaw)
    map_y = pos_y + x * np.sin(yaw) + y * np.cos(yaw)

    # Convert to grid coordinates
    grid_x = int(map_x / grid_resolution)
    grid_y = int(map_y / grid_resolution)

    return grid_x, grid_y

def detect_obstacles(robot_pose, cartesian_lidar_data, grid_resolution, threshold=2.0):
    """
    Detect obstacles within a specified threshold distance.
    
    :param robot_pose: The pose of the robot in the map frame.
    :param cartesian_lidar_data: Processed LIDAR data in Cartesian coordinates.
    :param grid_resolution: Resolution of the grid map.
    :param threshold: Distance threshold for obstacle detection in meters.
    :return: A list of grid coordinates (tuples) representing detected obstacles.
    """
    obstacles = []
    for x, y in cartesian_lidar_data:
        # Calculate the distance from the sensor
        distance = np.sqrt(x**2 + y**2)

        if distance <= threshold:
            # Convert Cartesian coordinates to map coordinates based on robot's pose
            grid_x, grid_y = convert_to_map_coordinates(robot_pose, (x, y), grid_resolution)
            
            # Add the grid coordinate to the list of obstacles
            obstacles.append((grid_x, grid_y))

    return list(set(obstacles))


def update_map_with_obstacles(obstacles, grid_size, grid_resolution):
    """
    Create or update a dynamic obstacle map/grid.

    :param obstacles: List of grid coordinates representing obstacles.
    :param grid_size: Size of the grid (width, height).
    :param grid_resolution: Resolution of the grid map.
    :return: Updated 2D grid map with obstacles marked.
    """
    # Create an empty grid
    grid_map = np.zeros(grid_size, dtype=int)

    # Mark obstacles in the grid
    for x, y in obstacles:
        # Check if the obstacle coordinates are within the grid bounds
        if 0 <= x < grid_size[0] and 0 <= y < grid_size[1]:
            grid_map[x, y] = 1  # Mark the cell as an obstacle (value 1)

    return grid_map


