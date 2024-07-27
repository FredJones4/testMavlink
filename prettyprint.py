import json

def print_pretty_dict(d, indent=4):
    """
    Prints the contents of a dictionary (and nested dictionaries) in a pretty format.

    Parameters:
    d (dict): The dictionary to print.
    indent (int): Number of spaces to use for indentation.
    """
    def print_dict(d, level=0):
        """Helper function to print dictionary contents."""
        for key, value in d.items():
            if isinstance(value, dict):
                print(' ' * (level * indent) + f"{key}:")
                print_dict(value, level + 1)
            else:
                print(' ' * (level * indent) + f"{key}: {value}")

    print_dict(d)

# Example usage:
data = {
    'raw_imu_error': 'temp item for now',
    'euler_angles': {
        'roll_deg': 0.3188048303127289,
        'pitch_deg': -0.03567584231495857,
        'yaw_deg': 0.008595028892159462,
        'timestamp_us': 4122355294000
    },
    'angular_velocity': {
        'roll_rad_s': 0.0045650978572666645,
        'pitch_rad_s': 0.0004399002646096051,
        'yaw_rad_s': 0.004965764936059713
    },
    'rc_status': {
        'is_available': False,
        'was_available_once': False,
        'signal_strength_percent': float('nan')
    }
}

print_pretty_dict(data)
