import numpy as np
import tf
import rospy
import warnings
warnings.filterwarnings("error")

def filter_noise(lidar_data):
    """
    Apply a median filter to reduce noise in LIDAR data.
    :param lidar_data: A numpy array of LIDAR measurements.
    :return: Filtered LIDAR data.
    """
    # Replace np.median with a different filter if needed
    rospy.loginfo(np.shape(lidar_data))
    rospy.loginfo(lidar_data[0])
    filtered_data = np.median(lidar_data, axis=1)
    rospy.loginfo(np.shape(filtered_data))
    rospy.loginfo(filtered_data[0])

    return filtered_data

def polar_to_cartesian(lidar_data, min, max):
    """
    Convert LIDAR data from polar to Cartesian coordinates.
    :param lidar_data: A numpy array of LIDAR measurements.
    :return: LIDAR data in Cartesian coordinates.
    """
    try:
        angles = np.linspace(min, max, len(lidar_data))
        distances = lidar_data
        # distances = [1 if ((len(lidar_data) / 4) < i <( 3 * (len(lidar_data) / 4))) else 0.5 if (len(lidar_data) / 4) < i else 3 for (i, v) in enumerate(lidar_data)]
        x = np.multiply(distances, np.cos(angles))
        y = np.multiply(distances, np.sin(angles))
        ret = np.column_stack((x, y))
        a = []
        for (rx, ry) in ret: 
            if rx != 0 and ry != 0:
                a.append((rx,ry))
        return np.array(a)
    except RuntimeWarning:
        return np.array([])
        


def process_lidar_data(raw_lidar_data, min, max):
    """
    Preprocess raw LIDAR data.
    :param raw_lidar_data: A numpy array of raw LIDAR measurements.
    :return: Processed LIDAR data in Cartesian coordinates.
    """
    # filtered_data = filter_noise(raw_lidar_data)
    # nd = np.array([r[1] for r in raw_lidar_data])
    cartesian_data = polar_to_cartesian(raw_lidar_data, min, max)
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
    grid_x = int(map_x / grid_resolution) + 2000
    grid_y = int(map_y / grid_resolution) + 2000

    return grid_x, grid_y

def detect_obstacles(robot_pose, cartesian_lidar_data, grid_resolution, threshold=3.5):
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


def update_map_with_obstacles(dynamic_obstacles, map, grid_size):
    """
    Update the map/grid with dynamic and static obstacle information.

    :param dynamic_obstacles: List of dynamic obstacle coordinates.
    :param static_obstacles: List of static obstacle coordinates.
    :param grid_size: Size of the grid (width, height).
    :return: Updated 2D grid map.
    """
    # Create an empty grid
    obstacles = []

    # Mark static and dynamic obstacles in the grid
    for obstacle_list in [dynamic_obstacles]:
        for x, y in obstacle_list:
            if 0 <= x < grid_size[0] and 0 <= y < grid_size[1]:
                xs = [x + i for i in range(-2, 3) if 0 <= x + i < 4000]
                ys = [y + i for i in range(-2, 3) if 0 <= y + i < 4000]
                for ax in xs:
                    for ay in ys:
                        obstacles.append((ax, ay)) # Mark the cell as an obstacle

    return np.array(obstacles)


