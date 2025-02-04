from scipy.spatial.transform import Rotation as R

def euler_to_quaternion(rx, ry, rz, degrees=True):
    """
    Convert Euler angles to a quaternion using scipy.
    
    Args:
    rx, ry, rz (float): Euler angles for roll (X-axis), pitch (Y-axis), and yaw (Z-axis).
    degrees (bool): Whether the input angles are in degrees. Default is True.

    Returns:
    tuple: Quaternion (qx, qy, qz, qw).
    """
    # Create rotation object
    rotation = R.from_euler('xyz', [rx, ry, rz], degrees=degrees)
    
    # Convert to quaternion
    quaternion = rotation.as_quat()  # Format: [qx, qy, qz, qw]
    
    return tuple(quaternion)

# Example usage
euler_angles = (87, 20, 0)  # RX, RY, RZ in degrees
quaternion = euler_to_quaternion(*euler_angles)
print("Quaternion:", quaternion)