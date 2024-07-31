"""
Results:

Listing Offboard class commands and methods:
Class: Offboard
    Method: __init__
    Method: _extract_result
    Method: _init_plugin
    Method: _setup_stub
    Method: is_active
    Attribute: name
    Method: set_acceleration_ned
    Method: set_actuator_control
    Method: set_attitude
    Method: set_attitude_rate
    Method: set_position_global
    Method: set_position_ned
    Method: set_position_velocity_acceleration_ned
    Method: set_position_velocity_ned
    Method: set_velocity_body
    Method: set_velocity_ned
    Method: start
    Method: stop

Listing Action class commands and methods:
Class: Action
    Method: __init__
    Method: _extract_result
    Method: _init_plugin
    Method: _setup_stub
    Method: arm
    Method: arm_force
    Method: disarm
    Method: do_orbit
    Method: get_maximum_speed
    Method: get_return_to_launch_altitude
    Method: get_takeoff_altitude
    Method: goto_location
    Method: hold
    Method: kill
    Method: land
    Attribute: name
    Method: reboot
    Method: return_to_launch
    Method: set_actuator
    Method: set_current_speed
    Method: set_maximum_speed
    Method: set_return_to_launch_altitude
    Method: set_takeoff_altitude
    Method: shutdown
    Method: takeoff
    Method: terminate
    Method: transition_to_fixedwing
    Method: transition_to_multicopter

Listing Telemetry class commands and methods:
Class: Telemetry
    Method: __init__
    Method: _extract_result
    Method: _init_plugin
    Method: _setup_stub
    Method: actuator_control_target
    Method: actuator_output_status
    Method: altitude
    Method: armed
    Method: attitude_angular_velocity_body
    Method: attitude_euler
    Method: attitude_quaternion
    Method: battery
    Method: camera_attitude_euler
    Method: camera_attitude_quaternion
    Method: distance_sensor
    Method: fixedwing_metrics
    Method: flight_mode
    Method: get_gps_global_origin
    Method: gps_info
    Method: ground_truth
    Method: heading
    Method: health
    Method: health_all_ok
    Method: home
    Method: imu
    Method: in_air
    Method: landed_state
    Attribute: name
    Method: odometry
    Method: position
    Method: position_velocity_ned
    Method: raw_gps
    Method: raw_imu
    Method: rc_status
    Method: scaled_imu
    Method: scaled_pressure
    Method: set_rate_actuator_control_target
    Method: set_rate_actuator_output_status
    Method: set_rate_altitude
    Method: set_rate_attitude_euler
    Method: set_rate_attitude_quaternion
    Method: set_rate_battery
    Method: set_rate_camera_attitude
    Method: set_rate_distance_sensor
    Method: set_rate_fixedwing_metrics
    Method: set_rate_gps_info
    Method: set_rate_ground_truth
    Method: set_rate_home
    Method: set_rate_imu
    Method: set_rate_in_air
    Method: set_rate_landed_state
    Method: set_rate_odometry
    Method: set_rate_position
    Method: set_rate_position_velocity_ned
    Method: set_rate_raw_imu
    Method: set_rate_rc_status
    Method: set_rate_scaled_imu
    Method: set_rate_unix_epoch_time
    Method: set_rate_velocity_ned
    Method: set_rate_vtol_state
    Method: status_text
    Method: unix_epoch_time
    Method: velocity_ned
    Method: vtol_state

"""


import inspect
from mavsdk.offboard import Offboard
from mavsdk.action import Action
from mavsdk.telemetry import Telemetry

def list_commands_and_methods(cls, indent=0):
    """
    Recursively list all commands and methods of a given class and its nested classes.
    
    Parameters:
    cls (type): The class to inspect.
    indent (int): The current indentation level for printing.
    """
    print(" " * indent + f"Class: {cls.__name__}")
    for name, member in inspect.getmembers(cls):
        if inspect.isclass(member) and member.__module__ == cls.__module__:
            list_commands_and_methods(member, indent + 4)
        elif inspect.isfunction(member) or inspect.ismethod(member):
            print(" " * (indent + 4) + f"Method: {name}")
        elif not name.startswith('_'):
            print(" " * (indent + 4) + f"Attribute: {name}")

def list_offboard_commands_and_methods():
    """
    List all commands and methods in MAVSDK's Offboard class.
    """
    print("Listing Offboard class commands and methods:")
    list_commands_and_methods(Offboard)

def list_action_commands_and_methods():
    """
    List all commands and methods in MAVSDK's Action class.
    """
    print("\nListing Action class commands and methods:")
    list_commands_and_methods(Action)

def list_telemetry_commands_and_methods():
    """
    List all commands and methods in MAVSDK's Telemetry class.
    """
    print("\nListing Telemetry class commands and methods:")
    list_commands_and_methods(Telemetry)

def list_all_commands_and_methods():
    """
    List all commands and methods for Offboard, Action, and Telemetry classes.
    """
    list_offboard_commands_and_methods()
    list_action_commands_and_methods()
    list_telemetry_commands_and_methods()

# Run the function to list the commands and methods
list_all_commands_and_methods()
