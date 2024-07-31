import numpy as np

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles to a rotation matrix.
    Args:
    roll (float): Roll angle in degrees.
    pitch (float): Pitch angle in degrees.
    yaw (float): Yaw angle in degrees.

    Returns:
    np.ndarray: Rotation matrix.
    """
    roll_rad = np.radians(roll)
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)
    
    R_z = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])
    R_y = np.array([
        [np.cos(pitch_rad), 0, -np.sin(pitch_rad)],
        [0, 1, 0],
        [np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), np.sin(roll_rad)],
        [0, -np.sin(roll_rad), np.cos(roll_rad)]
    ])

    R = R_z @ R_y @ R_x
    return R

def euler_to_rotation_matrix_chat_version(roll, pitch, yaw):
    """
    Convert Euler angles to a rotation matrix.
    Args:
    roll (float): Roll angle in degrees.
    pitch (float): Pitch angle in degrees.
    yaw (float): Yaw angle in degrees.

    Returns:
    np.ndarray: Rotation matrix.
    """
    roll_rad = np.radians(roll)
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)

    R_z = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])

    R_y = np.array([
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
        [0, 1, 0],
        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])

    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), -np.sin(roll_rad)],
        [0, np.sin(roll_rad), np.cos(roll_rad)]
    ])

    R = R_z @ R_y @ R_x
    return R

def compare_rotation_matrices(roll, pitch, yaw):
    """
    Compare the rotation matrices produced by the two functions.
    Args:
    roll (float): Roll angle in degrees.
    pitch (float): Pitch angle in degrees.
    yaw (float): Yaw angle in degrees.

    Returns:
    None
    """
    R1 = euler_to_rotation_matrix(roll, pitch, yaw)
    R2 = euler_to_rotation_matrix_chat_version(roll, pitch, yaw)
    
    print("Rotation Matrix from euler_to_rotation_matrix:")
    print(R1)
    print("\nRotation Matrix from euler_to_rotation_matrix_chat_version:")
    print(R2)
    
    # Compare matrices
    difference = np.abs(R1 - R2)
    max_difference = np.max(difference)
    
    print("\nDifference Matrix:")
    print(difference)
    print("\nMaximum Difference:", max_difference)

# Example angles
roll = 30
pitch = 45
yaw = 60

compare_rotation_matrices(roll, pitch, yaw)
