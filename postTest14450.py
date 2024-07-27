#!/usr/bin/env python3

"""
Caveat when attempting to run the examples in non-gps environments:

`drone.offboard.stop()` will return a `COMMAND_DENIED` result because it
requires a mode switch to HOLD, something that is currently not supported in a
non-gps environment.
"""

import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


async def request_rc_status_printout(drone):
    try:
        # Verify the exact method name
        if not hasattr(drone.telemetry, 'rc_status'):
            print("RC Status method not available.")
            return

        # Create a task to handle RC status data
        async def get_rc_status_data():
            async for rc_status in drone.telemetry.rc_status():
                # Print all attributes of the rc_status object
                print("RC Status attributes:", dir(rc_status))
                # Print the rc_status object itself to inspect its structure
                print(f"RC Status: {rc_status}")
                return  # Exit after first result

        # Run the get_rc_status_data coroutine with timeout
        try:
            await asyncio.wait_for(get_rc_status_data(), timeout=5)
        except asyncio.TimeoutError:
            print("Timeout occurred while waiting for RC status data.")
    except AttributeError as e:
        print(f"Error retrieving RC status data: {e}")

async def request_rc_status(drone):
    try:
        # Verify the exact method name
        if not hasattr(drone.telemetry, 'rc_status'):
            print("RC Status method not available.")
            return

        # Create a task to handle RC status data
        async def get_rc_status_data():
            async for rc_status in drone.telemetry.rc_status():
                # Print available attributes of the rc_status object
                print(f"RC Status: Available={rc_status.is_available}, "
                      f"Was Available Once={rc_status.was_available_once}, "
                      f"Signal Strength={rc_status.signal_strength_percent}%")
                return  # Exit after first result

        # Run the get_rc_status_data coroutine with timeout
        try:
            await asyncio.wait_for(get_rc_status_data(), timeout=5)
        except asyncio.TimeoutError:
            print("Timeout occurred while waiting for RC status data.")
    except AttributeError as e:
        print(f"Error retrieving RC status data: {e}")

async def request_raw_imu(drone): #NOTE: This likely does not work in simulation bc it's not offered in this simulation. "Some simulators or hardware setups might not send this data unless specific conditions are met." 
    # NOTE: https://discuss.px4.io/t/logging-high-resolution-imu-data-only/1673 for acutally logging the data at high rates.
    try:
        # Verify the exact method name
        if not hasattr(drone.telemetry, 'raw_imu'):
            print("Raw IMU method not available.")
            return

        # Create a task to handle raw_imu data
        async def get_raw_imu_data():
            async for imu in drone.telemetry.raw_imu():
                print(f"Raw IMU: Acceleration (FRD)={imu.acceleration_frd}, "
                      f"Gyroscope (FRD)={imu.gyroscope_frd}, "
                      f"Magnetometer (FRD)={imu.magnetometer_frd}")
                return  # Exit after first result

        # Run the get_raw_imu_data coroutine with timeout
        try:
            await asyncio.wait_for(get_raw_imu_data(), timeout=5)
        except asyncio.TimeoutError:
            print("Timeout occurred while waiting for raw IMU data.")
    except AttributeError as e:
        print(f"Error retrieving raw IMU data: {e}")

async def request_euler_angles(drone): # works, and is able to be run once :)
    try:
        # Request Euler angles once
        async for euler_angle in drone.telemetry.attitude_euler():
            print(f"Euler Angles: Roll={euler_angle.roll_deg} degrees, "
                  f"Pitch={euler_angle.pitch_deg} degrees, "
                  f"Yaw={euler_angle.yaw_deg} degrees, "
                  f"Timestamp={euler_angle.timestamp_us} µs")
            break  # Exit after first result
    except AttributeError:
        print("Euler Angle method not available.")



async def request_angular_velocity(drone): # works, and is able to be run once :)
    try:
        # Check if attitude_angular_velocity_body method exists
        if not hasattr(drone.telemetry, 'attitude_angular_velocity_body'):
            print("Attitude Angular Velocity Body method not available.")
            return

        # Request Angular velocity once
        async for angular_velocity in drone.telemetry.attitude_angular_velocity_body():
            print(f"Angular Velocity: Roll={angular_velocity.roll_rad_s} rad/s, "
                  f"Pitch={angular_velocity.pitch_rad_s} rad/s, "
                  f"Yaw={angular_velocity.yaw_rad_s} rad/s")
            break  # Exit after first result
    except AttributeError as e:
        print(f"Error retrieving angular velocity: {e}")


async def request_sensor_data(drone):
    async for health in drone.telemetry.health():
        print(f"Health: {health}")
        await asyncio.sleep(1)  # Adjust as needed


async def list_telemetry_methods(drone): # works
    # List all telemetry methods
    telemetry = drone.telemetry
    methods = [method for method in dir(telemetry) if callable(getattr(telemetry, method)) and not method.startswith("__")]
    print("Available telemetry methods:")
    for method in methods:
        print(method)

async def run():
    """ Does Offboard control using position NED coordinates. """

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
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Go 0m North, 0m East, -5m Down \
            within local coordinate system")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    await asyncio.sleep(10)


    # # Handle FixedwingMetrics for airspeed --- success; works in loop. TODO: have simply grab once for future loop 
    # # TODO: have just the data grabbed so all can be wrapped.
    # try:
    #     async for metrics in drone.telemetry.fixedwing_metrics():
    #         print(f"Airspeed: {metrics.airspeed_m_s} m/s")
    #         print(f"Throttle Percentage: {metrics.throttle_percentage}%")
    #         print(f"Climb Rate: {metrics.climb_rate_m_s} m/s")
    #         await asyncio.sleep(1)
    # except AttributeError:
    #     print("FixedwingMetrics method not available.")

    # async for rc_status in drone.telemetry.rc_status():
    #     print(f"RC Status: {rc_status}")
    #     await asyncio.sleep(1)



    # Request Euler angles and angular velocity once
    # await request_euler_angles(drone) # works, and can be run once :)
    # await request_angular_velocity(drone) 


    # Request attitude Euler angles, RC status, and raw IMU data once
    await request_rc_status(drone) # not working #TODO: fix
    # await request_raw_imu(drone)




    #     # List telemetry methods to verify method names
    # await list_telemetry_methods(drone)



    print("-- Go 5m North, 0m East, -5m Down \
            within local coordinate system, turn to face East")
    await drone.offboard.set_position_ned(
            PositionNedYaw(5.0, 0.0, -5.0, 90.0))
    await asyncio.sleep(10)

    print("-- Go 5m North, 10m East, -5m Down \
            within local coordinate system")
    await drone.offboard.set_position_ned(
            PositionNedYaw(5.0, 10.0, -5.0, 90.0))
    await asyncio.sleep(15)

    print("-- Go 0m North, 10m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 10.0, 0.0, 180.0))
    await asyncio.sleep(10)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                with error code: {error._result.result}")


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())