import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
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

async def collect_telemetry_data(drone):
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
            'acceleration_frd': 0, #TODO: debug why this won't work in sitl. Consider changing for HITL.
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

    # Run all the requests concurrently
    await asyncio.gather(
        request_rc_status(drone),
        request_raw_imu(drone),
        request_euler_angles(drone),
        request_angular_velocity(drone),
        request_airspeed(drone),
        request_position(drone)
    )

    return data

# Example usage
async def run():
    """Does Offboard control using position NED coordinates."""
    drone = System()
    await drone.connect(system_address="udp://:14540")

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
