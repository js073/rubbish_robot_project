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


def remove_invalid_measurements(lidar_data, min_distance, max_distance):
    """
    Remove invalid LIDAR measurements.
    :param lidar_data: A numpy array of LIDAR measurements.
    :param min_distance: Minimum valid distance.
    :param max_distance: Maximum valid distance.
    :return: LIDAR data with valid measurements only.
    """
    valid_data = lidar_data[(lidar_data[:, 1] > min_distance) & (lidar_data[:, 1] < max_distance)]
    return valid_data


def preprocess_lidar_data(raw_lidar_data, min_distance=0.2, max_distance=20.0):
    """
    Preprocess raw LIDAR data.
    :param raw_lidar_data: A numpy array of raw LIDAR measurements.
    :param min_distance: Minimum valid distance.
    :param max_distance: Maximum valid distance.
    :return: Processed LIDAR data in Cartesian coordinates.
    """
    filtered_data = filter_noise(raw_lidar_data)
    valid_data = remove_invalid_measurements(filtered_data, min_distance, max_distance)
    cartesian_data = polar_to_cartesian(valid_data)
    return cartesian_data

# Example usage
# raw_lidar_data = # Fetch raw LIDAR data from your sensor or simulation
# processed_data = preprocess_lidar_data(raw_lidar_data)
