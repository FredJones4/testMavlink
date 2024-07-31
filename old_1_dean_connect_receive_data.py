'''
How to use:
1. cd PX4-Autopilot
2. make px4_sitl jmavsim (or, even better, make px4_sitl gz_standard_vtol)
3. in a new terminal or in VSCode, start this script.
'''

'''
Available telemetry methods: (NOTE: velocity_body is available in C++, but not Python)
_extract_result
_init_plugin
_setup_stub
actuator_control_target
actuator_output_status
altitude
armed
attitude_angular_velocity_body
attitude_euler
attitude_quaternion
battery
camera_attitude_euler
camera_attitude_quaternion
distance_sensor
fixedwing_metrics
flight_mode
get_gps_global_origin
gps_info
ground_truth
heading
health
health_all_ok
home
imu
in_air
landed_state
odometry
position
position_velocity_ned
raw_gps
raw_imu
rc_status
scaled_imu
scaled_pressure
set_rate_actuator_control_target
set_rate_actuator_output_status
set_rate_altitude
set_rate_attitude_euler
set_rate_attitude_quaternion
set_rate_battery
set_rate_camera_attitude
set_rate_distance_sensor
set_rate_fixedwing_metrics
set_rate_gps_info
set_rate_ground_truth
set_rate_home
set_rate_imu
set_rate_in_air
set_rate_landed_state
set_rate_odometry
set_rate_position
set_rate_position_velocity_ned
set_rate_raw_imu
set_rate_rc_status
set_rate_scaled_imu
set_rate_unix_epoch_time
set_rate_velocity_ned
set_rate_vtol_state
status_text
unix_epoch_time
velocity_ned
vtol_state
'''
'''
Sample output:
/bin/python3 /home/cmhales/testMavlink/1_dean_connect_receive_data.py
WARNING: All log messages before absl::InitializeLog() is called are written to STDERR
I0000 00:00:1722376891.137142   12805 config.cc:230] gRPC experiments enabled: call_status_override_on_cancellation, event_engine_dns, event_engine_listener, http2_stats_fix, monitoring_experiment, pick_first_new, trace_record_callops, work_serializer_clears_time_cache
Waiting for drone to connect...
-- Connected to drone!
Waiting for drone to have a global position estimate...
-- Global position estimate OK
-- Arming
-- Setting initial setpoint
-- Starting offboard
-- Go 0m North, 0m East, -5m Down within local coordinate system
Collected telemetry data: {'raw_imu_temp': {'acceleration_frd': 0, 'gyroscope_frd': 0, 'magnetometer_frd': 0}, 'euler_angles': {'roll_deg': 0.1782805621623993, 'pitch_deg': 0.6407455205917358, 'yaw_deg': 0.09707130491733551, 'timestamp_us': 95011147000}, 'angular_velocity': {'roll_rad_s': -0.026740513741970062, 'pitch_rad_s': 0.021897081285715103, 'yaw_rad_s': 0.001281383796595037}, 'position': {'latitude_deg': 47.3977418, 'longitude_deg': 8.5455917, 'absolute_altitude_m': 493.02703857421875, 'relative_altitude_m': 5.033000469207764}, 'velocity_ned': {'north_m_s': 0.04999999701976776, 'east_m_s': 0.23999999463558197, 'down_m_s': 0.0}, 'airspeed': {'airspeed_m_s': 0.0, 'throttle_percentage': 0.4899999797344208, 'climb_rate_m_s': 0.006944111082702875}, 'rc_status': {'is_available': False, 'was_available_once': False, 'signal_strength_percent': nan}, 'velocity_body': {'body_x_m_s': np.float64(0.04959854071352511), 'body_y_m_s': np.float64(0.2400832078613634), 'body_z_m_s': np.float64(0.00018758769240825792)}}
-- Go 5m North, 0m East, -5m Down within local coordinate system, turn to face East
-- Go 5m North, 10m East, -5m Down within local coordinate system
-- Go 0m North, 10m East, 0m Down within local coordinate system, turn to face South
-- Stopping offboard
Collected telemetry data:
raw_imu_temp:
    acceleration_frd: 0
    gyroscope_frd: 0
    magnetometer_frd: 0
euler_angles:
    roll_deg: 0.34814807772636414
    pitch_deg: -1.5503171682357788
    yaw_deg: 179.96331787109375
    timestamp_us: 95046255000
angular_velocity:
    roll_rad_s: 0.00418394710868597
    pitch_rad_s: 0.03408871963620186
    yaw_rad_s: -0.002598187420517206
velocity_ned:
    north_m_s: -0.35999998450279236
    east_m_s: -0.019999999552965164
    down_m_s: 0.009999999776482582
position:
    latitude_deg: 47.3977462
    longitude_deg: 8.545727099999999
    absolute_altitude_m: 488.06201171875
    relative_altitude_m: 0.06800000369548798
rc_status:
    is_available: False
    was_available_once: False
    signal_strength_percent: nan
airspeed:
    airspeed_m_s: 0.0
    throttle_percentage: 0.4899999797344208
    climb_rate_m_s: -0.017796244472265244
velocity_body:
    body_x_m_s: 0.3601482316757741
    body_y_m_s: 0.019829821828249355
    body_z_m_s: 0.00013493232310625447
'''
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import json
import numpy as np

LOCAL_HOST_TEST = "udp://:14540"
CURR_USB_CONNECTION = "/dev/ttyACM1"

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





def calculate_velocity_body(velocity_ned, euler_angles):
    """
    Calculate body frame velocities from NED frame velocities and Euler angles.
    Args:
    velocity_ned (dict): Dictionary containing NED frame velocities.
    euler_angles (dict): Dictionary containing Euler angles.

    Returns:
    dict: Dictionary containing body frame velocities.
    """
    R = euler_to_rotation_matrix(
        euler_angles['roll_deg'],
        euler_angles['pitch_deg'],
        euler_angles['yaw_deg']
    )

    ned_velocity = np.array([
        velocity_ned['north_m_s'],
        velocity_ned['east_m_s'],
        velocity_ned['down_m_s']
    ])

    body_velocity = R @ ned_velocity

    return {
        'body_x_m_s': body_velocity[0],
        'body_y_m_s': body_velocity[1],
        'body_z_m_s': body_velocity[2]
    }

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

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles to a rotation matrix.
    Based off of p16 (page 24 of 397, Figure 2.7) of Dr. Beard's UAV Book:
    https://www.dropbox.com/scl/fi/uwa7xwpb9imxziutfem6z/uavbook.pdf?rlkey=efpq4vy3ynizf427gexu7rs6e&e=1&dl=0
    
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
    # R_v_v1
    R_z = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])
    # R_v1_v2
    R_y = np.array([
        [np.cos(pitch_rad), 0, -np.sin(pitch_rad)],
        [0, 1, 0],
        [np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])
    # R_v2^b
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), np.sin(roll_rad)],
        [0, -np.sin(roll_rad), np.cos(roll_rad)]
    ])

    R = R_z @ R_y @ R_x
    return R

def calculate_velocity_body(velocity_ned, euler_angles):
    """
    Calculate body frame velocities from NED frame velocities and Euler angles.
    Args:
    velocity_ned (dict): Dictionary containing NED frame velocities.
    euler_angles (dict): Dictionary containing Euler angles.

    Returns:
    dict: Dictionary containing body frame velocities.
    """
    R = euler_to_rotation_matrix(
        euler_angles['roll_deg'],
        euler_angles['pitch_deg'],
        euler_angles['yaw_deg']
    )

    ned_velocity = np.array([
        velocity_ned['north_m_s'],
        velocity_ned['east_m_s'],
        velocity_ned['down_m_s']
    ])

    body_velocity = R @ ned_velocity

    return {
        'body_x_m_s': body_velocity[0],
        'body_y_m_s': body_velocity[1],
        'body_z_m_s': body_velocity[2]
    }

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

def calculate_velocity_body(velocity_ned, euler_angles):
    """
    Calculate body frame velocities from NED frame velocities and Euler angles.
    Args:
    velocity_ned (dict): Dictionary containing NED frame velocities.
    euler_angles (dict): Dictionary containing Euler angles.

    Returns:
    dict: Dictionary containing body frame velocities.
    """
    R = euler_to_rotation_matrix(
        euler_angles['roll_deg'],
        euler_angles['pitch_deg'],
        euler_angles['yaw_deg']
    )

    ned_velocity = np.array([
        velocity_ned['north_m_s'],
        velocity_ned['east_m_s'],
        velocity_ned['down_m_s']
    ])

    body_velocity = R @ ned_velocity

    return {
        'body_x_m_s': body_velocity[0],
        'body_y_m_s': body_velocity[1],
        'body_z_m_s': body_velocity[2]
    }

async def collect_telemetry_data(drone):
    """
    Produces the parameters needed for control data to calculate angle of attack, the sideslip angle, and the state as a whole.
    Parameters:
    drone(System): The MAV being flown.
    
    Returns:
    data (dict): The dictionary of all received state data, plus some helpful debugger information about RC, etc.
    """
    data = {}

    async def request_rc_status(drone):
        try:
            if not hasattr(drone.telemetry, 'rc_status'):
                data['rc_status_error'] = "RC Status method not available."
                return
            
            async for rc_status in drone.telemetry.rc_status():
                data['rc_status'] = {
                    'is_available': rc_status.is_available,
                    'was_available_once': rc_status.was_available_once,
                    'signal_strength_percent': rc_status.signal_strength_percent
                }
                break  # Exit after first result
        except AttributeError as e:
            data['rc_status_error'] = str(e)

    async def request_raw_imu(drone):
        data['raw_imu_temp'] = {
            'acceleration_frd': 0, # TODO: get raw_imu data to actually work in simulation.
            'gyroscope_frd': 0,
            'magnetometer_frd': 0
        }

    async def request_euler_angles(drone):
        try:
            async for euler_angle in drone.telemetry.attitude_euler():
                data['euler_angles'] = {
                    'roll_deg': euler_angle.roll_deg,
                    'pitch_deg': euler_angle.pitch_deg,
                    'yaw_deg': euler_angle.yaw_deg,
                    'timestamp_us': euler_angle.timestamp_us
                }
                break  # Exit after first result
        except AttributeError:
            data['euler_angles_error'] = "Euler Angle method not available."

    async def request_angular_velocity(drone):
        try:
            if not hasattr(drone.telemetry, 'attitude_angular_velocity_body'):
                data['angular_velocity_error'] = "Attitude Angular Velocity Body method not available."
                return
            
            async for angular_velocity in drone.telemetry.attitude_angular_velocity_body():
                data['angular_velocity'] = {
                    'roll_rad_s': angular_velocity.roll_rad_s,
                    'pitch_rad_s': angular_velocity.pitch_rad_s,
                    'yaw_rad_s': angular_velocity.yaw_rad_s
                }
                break  # Exit after first result
        except AttributeError as e:
            data['angular_velocity_error'] = str(e)

    async def request_airspeed(drone):
        try:
            async for metrics in drone.telemetry.fixedwing_metrics():
                data['airspeed'] = {
                    'airspeed_m_s': metrics.airspeed_m_s,
                    'throttle_percentage': metrics.throttle_percentage,
                    'climb_rate_m_s': metrics.climb_rate_m_s
                }
                break  # Exit after first result
        except AttributeError:
            data['airspeed_error'] = "FixedwingMetrics method not available."

    async def request_position(drone):
        try:
            async for position in drone.telemetry.position():
                data['position'] = {
                    'latitude_deg': position.latitude_deg,
                    'longitude_deg': position.longitude_deg,
                    'absolute_altitude_m': position.absolute_altitude_m,
                    'relative_altitude_m': position.relative_altitude_m
                }
                break  # Exit after first result
        except AttributeError:
            data['position_error'] = "Position method not available."

    async def request_velocity_ned(drone):
        try:
            async for velocity in drone.telemetry.velocity_ned():
                data['velocity_ned'] = {
                    'north_m_s': velocity.north_m_s,
                    'east_m_s': velocity.east_m_s,
                    'down_m_s': velocity.down_m_s
                }
                break  # Exit after first result
        except AttributeError:
            data['velocity_ned_error'] = "Velocity NED method not available."

    # Run all the requests concurrently
    await asyncio.gather(
        request_rc_status(drone),
        request_raw_imu(drone),
        request_euler_angles(drone),
        request_angular_velocity(drone),
        request_airspeed(drone),
        request_position(drone),
        request_velocity_ned(drone)
    )

    # Calculate body frame velocities if both velocity_ned and euler_angles are available
    if 'velocity_ned' in data and 'euler_angles' in data:
        data['velocity_body'] = calculate_velocity_body(data['velocity_ned'], data['euler_angles'])


    return data



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

# Example usage
async def run():
    """Does Offboard control using position NED coordinates."""
    drone = System()
    
    if not await setup_mavlink_offboard(drone):
        return

    print("-- Go 0m North, 0m East, -5m Down within local coordinate system")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    await asyncio.sleep(10)

    data = await collect_telemetry_data(drone)
    print("Collected telemetry data:", data)

    print("-- Go 5m North, 0m East, -5m Down within local coordinate system, turn to face East")
    await drone.offboard.set_position_ned(PositionNedYaw(5.0, 0.0, -5.0, 90.0))
    await asyncio.sleep(10)

    print("-- Go 5m North, 10m East, -5m Down within local coordinate system")
    await drone.offboard.set_position_ned(PositionNedYaw(5.0, 10.0, -5.0, 90.0))
    await asyncio.sleep(15)

    print("-- Go 0m North, 10m East, 0m Down within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 10.0, 0.0, 180.0))
    await asyncio.sleep(10)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    data = await collect_telemetry_data(drone)
    print("Collected telemetry data:")
    print_pretty_dict(data)

# Run the asyncio event loop
if __name__ == "__main__":
    asyncio.run(run())
