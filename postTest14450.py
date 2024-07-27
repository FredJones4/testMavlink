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

async def request_sensor_data(drone):
    async for health in drone.telemetry.health():
        print(f"Health: {health}")
        await asyncio.sleep(1)  # Adjust as needed

async def request_telemetry_data(drone):
    async for airspeed in drone.telemetry.airspeed():
        print(f"Airspeed: {airspeed}")
        await asyncio.sleep(1)

    async for position in drone.telemetry.position():
        print(f"Position: {position}")
        await asyncio.sleep(1)

    async for attitude in drone.telemetry.attitude_euler():
        print(f"Attitude: {attitude}")
        await asyncio.sleep(1)

    async for velocity in drone.telemetry.velocity_ned():
        print(f"Velocity NED: {velocity}")
        await asyncio.sleep(1)


    async for rc_status in drone.telemetry.rc_status():
        print(f"RC Status: {rc_status}")
        await asyncio.sleep(1)

async def request_accelerometer_data(drone):
    async for imu in drone.telemetry.raw_imu():
        print(f"Accelerometer: {imu.acceleration_frd}")
        await asyncio.sleep(1)  # Adjust as needed


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


    # Handle FixedwingMetrics for airspeed
    try:
        async for metrics in drone.telemetry.fixedwing_metrics():
            print(f"Airspeed: {metrics.airspeed_m_s} m/s")
            print(f"Throttle Percentage: {metrics.throttle_percentage}%")
            print(f"Climb Rate: {metrics.climb_rate_m_s} m/s")
            await asyncio.sleep(1)
    except AttributeError:
        print("FixedwingMetrics method not available.")

    async for rc_status in drone.telemetry.rc_status():
        print(f"RC Status: {rc_status}")
        await asyncio.sleep(1)



    telemetry_task = asyncio.ensure_future(request_telemetry_data(drone))
    accelerometer_task = asyncio.ensure_future(request_accelerometer_data(drone))

    await asyncio.gather(telemetry_task, accelerometer_task)

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