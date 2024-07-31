import asyncio
from mavsdk import System
from mavsdk.offboard import Offboard, PositionNedYaw, OffboardError
from mavsdk.action import Action
from mavsdk.telemetry import Telemetry
import time

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
    import numpy as np

    def euler_to_rotation_matrix(roll, pitch, yaw):
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

        return R_z @ R_y @ R_x

    R = euler_to_rotation_matrix(euler_angles['roll_deg'], euler_angles['pitch_deg'], euler_angles['yaw_deg'])
    velocity_body = np.dot(R, np.array([
        velocity_ned['north_m_s'],
        velocity_ned['east_m_s'],
        -velocity_ned['down_m_s']
    ]))
    
    return {
        'body_x_m_s': velocity_body[0],
        'body_y_m_s': velocity_body[1],
        'body_z_m_s': velocity_body[2]
    }

async def collect_telemetry_data(drone):
    """
    Collects telemetry data from the drone.
    Parameters:
    drone (System): The MAV being flown.
    
    Returns:
    data (dict): The dictionary of all received state data.
    """
    t1 = time.time()
    data = {}

    # async def request_heading(drone):
    #     """
    #     See the following site for explanation:

    #     """
    #     try:
    #         heading_iter = drone.telemetry.heading()
    #         heading = await asyncio.wait_for(heading_iter.__anext__(), timeout=0.1)
    #         data['heading'] = {
    #             'heading_deg': heading
    #         }
    #     except asyncio.TimeoutError:
    #         data['heading'] = {
    #             'heading_deg': 0
    #         }
    # async def request_raw_gps(drone):
    #     """
    #     See the following site for exaplanation of attributes:
    #     https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_raw_gps.html
    #     NOTE: Adding this function to the telemetry adds 0.1 seconds to the server request.
    #     """
    #     try:
    #         raw_gps_iter = drone.telemetry.raw_gps()
    #         raw_gps = await asyncio.wait_for(raw_gps_iter.__anext__(), timeout=0.1)
            
            
    #         data['raw_gps'] = {
    #             'fix_type': getattr(raw_gps, 'fix_type', 'N/A'),
    #             'satellites_visible': getattr(raw_gps, 'satellites_visible', 'N/A'),
    #             'latitude_deg': getattr(raw_gps, 'latitude_deg', 'N/A'),
    #             'longitude_deg': getattr(raw_gps, 'longitude_deg', 'N/A'),
    #             'altitude_m': getattr(raw_gps, 'altitude_m', 'N/A')
    #         }
    #     except asyncio.TimeoutError:
    #         data['raw_gps'] = {
    #             'fix_type': 'N/A',
    #             'satellites_visible': 'N/A',
    #             'latitude_deg': 'N/A',
    #             'longitude_deg': 'N/A',
    #             'altitude_m': 'N/A'
    #         }


    # async def request_scaled_imu(drone):
    #     try:
    #         scaled_imu_iter = drone.telemetry.scaled_imu()
    #         imu = await asyncio.wait_for(scaled_imu_iter.__anext__(), timeout=0.1)
    #         data['scaled_imu'] = {
    #             'accelerometer_x': imu.accelerometer_x,
    #             'accelerometer_y': imu.accelerometer_y,
    #             'accelerometer_z': imu.accelerometer_z,
    #             'gyroscope_x': imu.gyroscope_x,
    #             'gyroscope_y': imu.gyroscope_y,
    #             'gyroscope_z': imu.gyroscope_z,
    #             'magnetometer_x': imu.magnetometer_x,
    #             'magnetometer_y': imu.magnetometer_y,
    #             'magnetometer_z': imu.magnetometer_z
    #         }
    #     except asyncio.TimeoutError:
    #         data['scaled_imu'] = {
    #             'accelerometer_x': 0,
    #             'accelerometer_y': 0,
    #             'accelerometer_z': 0,
    #             'gyroscope_x': 0,
    #             'gyroscope_y': 0,
    #             'gyroscope_z': 0,
    #             'magnetometer_x': 0,
    #             'magnetometer_y': 0,
    #             'magnetometer_z': 0
    #         }

    async def request_odometry(drone):

        """
        https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_quaternion.html


        Compare direction of odometry body velocity versus velocity_body's:

        odometry:
    time_usec: 1722392877786000
    frame_id: BODY_NED
    child_frame_id: BODY_NED
    position_body:
        x_m: 0.37078380584716797
        y_m: 10.01797103881836
        z_m: -0.07206083834171295
    quaternion:
        w: 0.0005066042067483068
        x: 0.002567560877650976
        y: 0.003984380513429642
        z: 0.9999887347221375
    velocity_body:
        x_m_s: -0.3913387060165405
        y_m_s: -0.013884179294109344
        z_m_s: 0.07221990823745728
    angular_velocity_body:
        roll_rad_s: 0.01406790129840374
        pitch_rad_s: 0.03159700706601143
        yaw_rad_s: -0.003555304603651166
    pose_covariance: Covariance: [covariance_matrix: [0.007349032908678055, nan, nan, nan, nan, nan, 0.007357769645750523, nan, nan, nan, nan, 0.01288561336696148, nan, nan, nan, 9.49851528275758e-06, nan, nan, 9.452327503822744e-06, nan, 0.00014645607734564692]]
    velocity_covariance: Covariance: [covariance_matrix: [0.0033022805582731962, nan, nan, nan, nan, nan, 0.0033262460492551327, nan, nan, nan, nan, 0.0024495150428265333, nan, nan, nan, nan, nan, nan, nan, nan, nan]]
velocity_ned:
    north_m_s: -0.38999998569488525
    east_m_s: -0.009999999776482582
    down_m_s: 0.07000000029802322
velocity_body:
    body_x_m_s: 0.38959321477451403
    body_y_m_s: 0.009086912978266703
    body_z_m_s: -0.07235014784676254
time_spent: 0.04288601875305176
        """
        try:
            odometry_iter = drone.telemetry.odometry()
            odometry = await asyncio.wait_for(odometry_iter.__anext__(), timeout=0.1)
            data['odometry'] = {
                # 'time_usec': odometry.time_usec,  # Timestamp in microseconds
                # 'frame_id': odometry.frame_id,  # Coordinate frame of reference for the pose data
                # 'child_frame_id': odometry.child_frame_id,  # Coordinate frame of reference for the velocity
                'position_body': {
                    'x_m': odometry.position_body.x_m,  # Position in body frame, X axis
                    'y_m': odometry.position_body.y_m,  # Position in body frame, Y axis
                    'z_m': odometry.position_body.z_m   # Position in body frame, Z axis
                },
                # 'quaternion': {
                #     'w': odometry.q.w,  # Quaternion component w
                #     'x': odometry.q.x,  # Quaternion component x
                #     'y': odometry.q.y,  # Quaternion component y
                #     'z': odometry.q.z   # Quaternion component z
                # },
                'velocity_body': {
                    'x_m_s': odometry.velocity_body.x_m_s,  # Linear velocity in body frame, X axis
                    'y_m_s': odometry.velocity_body.y_m_s,  # Linear velocity in body frame, Y axis
                    'z_m_s': odometry.velocity_body.z_m_s   # Linear velocity in body frame, Z axis
                },
                'angular_velocity_body': {
                    'roll_rad_s': odometry.angular_velocity_body.roll_rad_s,  # Angular velocity around roll axis
                    'pitch_rad_s': odometry.angular_velocity_body.pitch_rad_s,  # Angular velocity around pitch axis
                    'yaw_rad_s': odometry.angular_velocity_body.yaw_rad_s   # Angular velocity around yaw axis
                },
                # 'pose_covariance': odometry.pose_covariance,  # Pose covariance matrix
                # 'velocity_covariance': odometry.velocity_covariance  # Velocity covariance matrix
            }
        except asyncio.TimeoutError:
            data['odometry'] = {
                # 'time_usec': 0,
                # 'frame_id': 0,
                # 'child_frame_id': 0,
                'position_body': {
                    'x_m': 0,
                    'y_m': 0,
                    'z_m': 0
                },
                # 'quaternion': {
                #     'w': 0,
                #     'x': 0,
                #     'y': 0,
                #     'z': 0
                # },
                'velocity_body': {
                    'x_m_s': 0,
                    'y_m_s': 0,
                    'z_m_s': 0
                },
                'angular_velocity_body': {
                    'roll_rad_s': 0,
                    'pitch_rad_s': 0,
                    'yaw_rad_s': 0
                },
                # 'pose_covariance': [0] * 6,  # Assuming a 6-element covariance matrix
                # 'velocity_covariance': [0] * 6  # Assuming a 6-element covariance matrix
            }
        except AttributeError as e:
            data['odometry_error'] = str(e)

    async def request_imu(drone):
        try:
            imu_iter = drone.telemetry.imu()
            imu = await asyncio.wait_for(imu_iter.__anext__(), timeout=0.1)
            data['imu'] = {
                'acceleration_frd': imu.acceleration_frd,  # Acceleration in FRD frame
                # 'angular_velocity_frd': imu.angular_velocity_frd,  # Angular velocity in FRD frame
                # 'magnetic_field_frd': imu.magnetic_field_frd,  # Magnetic field in FRD frame
                # 'temperature_degc': imu.temperature_degc,  # Temperature in degrees Celsius
                # 'timestamp_us': imu.timestamp_us  # Timestamp in microseconds
            }
        except asyncio.TimeoutError:
            data['imu'] = {
                'acceleration_frd': {'x': 0, 'y': 0, 'z': 0},
                # 'angular_velocity_frd': {'roll_rad_s': 0, 'pitch_rad_s': 0, 'yaw_rad_s': 0},
                # 'magnetic_field_frd': {'x': 0, 'y': 0, 'z': 0},
                # 'temperature_degc': 0,
                # 'timestamp_us': 0
            }
        except AttributeError as e:
            data['imu_error'] = str(e)

    # async def request_euler_angles(drone):
    #     try:
    #         euler_angles_iter = drone.telemetry.attitude_euler()
    #         euler_angle = await asyncio.wait_for(euler_angles_iter.__anext__(), timeout=0.1)
    #         data['euler_angles'] = {
    #             'roll_deg': euler_angle.roll_deg,
    #             'pitch_deg': euler_angle.pitch_deg,
    #             'yaw_deg': euler_angle.yaw_deg,
    #             'timestamp_us': euler_angle.timestamp_us
    #         }
    #     except asyncio.TimeoutError:
    #         data['euler_angles'] = {
    #             'roll_deg': 0,
    #             'pitch_deg': 0,
    #             'yaw_deg': 0,
    #             'timestamp_us': 0
    #         }

    # async def request_angular_velocity(drone):
    #     try:
    #         angular_velocity_iter = drone.telemetry.attitude_angular_velocity_body()
    #         angular_velocity = await asyncio.wait_for(angular_velocity_iter.__anext__(), timeout=0.1)
    #         data['angular_velocity'] = {
    #             'roll_rad_s': angular_velocity.roll_rad_s,
    #             'pitch_rad_s': angular_velocity.pitch_rad_s,
    #             'yaw_rad_s': angular_velocity.yaw_rad_s
    #         }
    #     except asyncio.TimeoutError:
    #         data['angular_velocity'] = {
    #             'roll_rad_s': 0,
    #             'pitch_rad_s': 0,
    #             'yaw_rad_s': 0
    #         }

    # async def request_airspeed(drone):
    #     try:
    #         airspeed_iter = drone.telemetry.fixedwing_metrics()
    #         metrics = await asyncio.wait_for(airspeed_iter.__anext__(), timeout=0.1)
    #         data['airspeed'] = {
    #             'airspeed_m_s': metrics.airspeed_m_s,
    #             'throttle_percentage': metrics.throttle_percentage,
    #             'climb_rate_m_s': metrics.climb_rate_m_s
    #         }
    #     except asyncio.TimeoutError:
    #         data['airspeed'] = {
    #             'airspeed_m_s': 0,
    #             'throttle_percentage': 0,
    #             'climb_rate_m_s': 0
    #         }


    async def request_airspeed(drone):
        """
        https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_fixedwing_metrics.html
        Interesting note: this function timed out when the scaled_imu function commented out above tried to run.
        """
        try:
            airspeed_iter = drone.telemetry.fixedwing_metrics()
            try:
                metrics = await asyncio.wait_for(airspeed_iter.__anext__(), timeout=0.1)
                data['airspeed'] = {
                    'airspeed_m_s': metrics.airspeed_m_s,
                    # 'throttle_percentage': metrics.throttle_percentage,
                    # 'climb_rate_m_s': metrics.climb_rate_m_s
                }
            except asyncio.TimeoutError:
                data['airspeed'] = {
                    'airspeed_m_s': 0,
                    # 'throttle_percentage': 0,
                    # 'climb_rate_m_s': 0
                }
        except AttributeError:
            data['airspeed_error'] = "FixedwingMetrics method not available."

    # async def request_position(drone):
    #     """
    #         https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_position.html
    #     """
    #     try:

    #         position_iter = drone.telemetry.position()
    #         position = await asyncio.wait_for(position_iter.__anext__(), timeout=0.1)
    #         data['position'] = {
    #             'latitude_deg': position.latitude_deg,
    #             'longitude_deg': position.longitude_deg,
    #             'absolute_altitude_m': position.absolute_altitude_m,
    #             'relative_altitude_m': position.relative_altitude_m
    #         }
    #     except asyncio.TimeoutError:
    #         data['position'] = {
    #             'latitude_deg': 0,
    #             'longitude_deg': 0,
    #             'absolute_altitude_m': 0,
    #             'relative_altitude_m': 0
    #         }

    # async def request_velocity_ned(drone):
    #     """
    #     https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_velocity_ned.html

    #     Interestingly, there is documenation for velocity_body, but the function does not work, and python does not list it as an existing function.
    #     https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_velocity_body.html
    #     """
    #     try:
    #         velocity_ned_iter = drone.telemetry.velocity_ned()
    #         velocity = await asyncio.wait_for(velocity_ned_iter.__anext__(), timeout=0.1)
    #         data['velocity_ned'] = {
    #             'north_m_s': velocity.north_m_s,
    #             'east_m_s': velocity.east_m_s,
    #             'down_m_s': velocity.down_m_s
    #         }
    #     except asyncio.TimeoutError:
    #         data['velocity_ned'] = {
    #             'north_m_s': 0,
    #             'east_m_s': 0,
    #             'down_m_s': 0
    #         }

    # Run all the requests concurrently
    await asyncio.gather(
        # request_heading(drone),
        # request_raw_gps(drone),
        # request_scaled_imu(drone),
        request_odometry(drone),
        request_imu(drone),
        # request_euler_angles(drone),
        # request_angular_velocity(drone),
        request_airspeed(drone),
        # request_position(drone),
        # request_velocity_ned(drone)
    )

    # if 'velocity_ned' in data and 'euler_angles' in data:
    #     data['velocity_body'] = calculate_velocity_body(data['velocity_ned'], data['euler_angles'])
    # else:
    #     data['velocity_body'] = {
    #         'body_x_m_s': 0,
    #         'body_y_m_s': 0,
    #         'body_z_m_s': 0
    #     }
    t2 = time.time()
    data['time_spent'] = t2-t1
    return data

async def setup_mavlink_offboard(drone, curr_conn="udp://:14540"):
    """
    Abstracts away the setup of the MAVSDK MAVLink protocols.

    Parameters:
    drone (System()): The drone setup.
    curr_conn (str): Determines how MAVLink is connected. Defaults to local host 14540.

    Returns:
    bool: True if setup was successful, False otherwise.
    """
    await drone.connect(system_address=curr_conn)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
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
