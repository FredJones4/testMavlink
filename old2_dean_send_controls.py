"""
write a function that sets actuator control given the following variable inputs, setting variables that go outside of bounds to the closer limit (ex -1.1 to -1, 1.2 to 1)

Position 1 is Elevator [-1,1]
2 is alieron [-1,1]
3 is rudder [-1,1]
4 is motor 3 front motor [0,1]
5 is motor 2 rear motor [0,1]
6 is motor 4 starboard rear motor [0,1]
7 is motor 1 starboard front motor [0,1]
8 is motor 5 forward propulsion motor [0,1]

input_control = ActuatorControlGroup(controls=[Elevator,alieron,rudder, front motor, rear motor, starboard rear motor, starboard front motor, forward propulsion motor])

note: we are testing. It may be necessary to have 2 copies of the same control group input in setting acutator control. Set an input boolean where, if false, 1 control group is used. if not, then 2 contorl groups, as seen in documentation.


Note the script in context:

"""
##################################################################################################################################
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, ActuatorControl, ActuatorControlGroup, PositionNedYaw
import json

LOCAL_HOST_TEST = "udp://:14540"
CURR_USB_CONNECTION = "/dev/ttyACM1"
####################################################################################################################################
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

#######################################################################################################################################################

def clamp(value, min_value, max_value):
    """ Clamps the value between min_value and max_value. """
    return max(min_value, min(value, max_value))

def create_actuator_control_group(elevator, aileron, rudder, starboard_front_motor, rear_motor, front_motor, starboard_rear_motor, forward_propulsion_motor):
    """ Creates an ActuatorControlGroup with clamped values. """
    return ActuatorControlGroup(
        controls=[
            clamp(elevator, -1, 1),
            clamp(aileron, -1, 1),
            clamp(rudder, -1, 1),
            clamp(starboard_front_motor, 0, 1),
            clamp(rear_motor, 0, 1),
            clamp(front_motor, 0, 1),
            clamp(starboard_rear_motor, 0, 1),
            clamp(forward_propulsion_motor, 0, 1)
        ]
    )

async def set_actuator_control_homemade(drone, elevator, aileron, rudder, starboard_front_motor4, rear_motor2, front_motor3, starboard_rear_motor1, forward_propulsion_motor5, use_two_groups=False):
    """ Sets the actuator control with optional two control groups.
      NOTE: the order is set according to current test actuator outputs on QGroundControl."""
    control_group = create_actuator_control_group(elevator, aileron, rudder, starboard_front_motor4, rear_motor2, front_motor3, starboard_rear_motor1, forward_propulsion_motor5)

    if use_two_groups:
        control_group2 = create_actuator_control_group(elevator, aileron, rudder, starboard_front_motor4, rear_motor2, front_motor3, starboard_rear_motor1, forward_propulsion_motor5)
        control_groups = [control_group, control_group2]
    else:
        control_groups = [control_group]

    try:
        await drone.offboard.set_actuator_control(ActuatorControl(groups=control_groups))
    except OffboardError as e:
        print(f"Setting actuator control failed with error: {e}")
    await asyncio.sleep(0.1)
#######################################################################################################################

async def setup_mavlink_offboard(drone, curr_conn=LOCAL_HOST_TEST):
    """
    Abstracts away the setup of the MAVSDK MAVLink protocols.

    Parameters:
    drone (System()): The drone setup.
    curr_con(string): determines how mavlink is connected. Defaults to local host 14450.

    Returns:
    True / False (bool): Determine if setup was successful or not.
    """
    await drone.connect(system_address=curr_conn)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return False

    return True
###################################################################################################################################
# Rename the locally defined function for clarity


async def set_actuator_control_test(drone, elevator, aileron, rudder, starboard_front_motor, rear_motor, front_motor, starboard_rear_motor, forward_propulsion_motor, use_two_groups=False):
    """ Testable version of set_actuator_control function. """
    return await set_actuator_control_homemade(drone, elevator, aileron, rudder, starboard_front_motor, rear_motor, front_motor, starboard_rear_motor, forward_propulsion_motor, use_two_groups)

# Unit tests for set_actuator_control
async def test_set_actuator_control():
    """ Tests for set_actuator_control function. """
    drone = System()

    # Mock the drone connection and setup
    await setup_mavlink_offboard(drone)

    print("Running test cases...")

    # Test case 1: Normal input
    print("Test case 1: Normal input")
    await set_actuator_control_test(drone, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5)
    
    # Test case 2: Inputs outside bounds
    print("Test case 2: Inputs outside bounds")
    await set_actuator_control_test(drone, 1.2, -1.5, 0.7, 1.5, -0.2, 0.8, 0.9, 1.1)
    
    # Test case 3: Using two control groups
    print("Test case 3: Using two control groups")
    await set_actuator_control_test(drone, 0.3, -0.3, 0.0, 0.1, 0.2, 0.9, 0.4, 0.0, use_two_groups=True)
    
    # Test case 4: All values at bounds
    print("Test case 4: All values at bounds")
    await set_actuator_control_test(drone, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

# Main function to run tests
async def run_tests():
    """ Run all test cases. """
    await test_set_actuator_control()

# Run the asyncio event loop for tests
if __name__ == "__main__":
    asyncio.run(run_tests())
