"""
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
    """

import inspect
from mavsdk.offboard import Offboard
from mavsdk.action import Action

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

def list_offboard_and_action_commands():
    """
    List all commands and methods in MAVSDK's Offboard and Action classes.
    """
    print("Listing Offboard class commands and methods:")
    list_commands_and_methods(Offboard)

    print("\nListing Action class commands and methods:")
    list_commands_and_methods(Action)

# Run the function to list the commands and methods
list_offboard_and_action_commands()
