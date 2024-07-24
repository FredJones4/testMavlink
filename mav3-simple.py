import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityBodyYawspeed)

async def run():
    # Connect to the Pixhawk
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyS1:57600")

    # Wait for the drone to connect and print heartbeat
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Connected to drone !")
            break

    # Fetch and print telemetry data
    asyncio.ensure_future(print_telemetry(drone))

    # Fetch and print accelerometer data
    asyncio.ensure_future(print_accelerometer(drone))

    # Offboard control commands
    try:
        await setup_offboard_control(drone)
    except OffboardError as error:
        print(f"Offboard mode error: {error}")

async def print_telemetry(drone):
    async for position in drone.telemetry.position():
        print(f"Latitude: {position.latitude_deg}, Longitude: {position.longitude_deg}, Altitude: {position.relative_altitude_m}")

async def print_accelerometer(drone):
    async for imu in drone.telemetry.raw_imu():
        print(f"Accelerometer: [X: {imu.acceleration_x_m_s2}, Y: {imu.acceleration_y_m_s2}, Z: {imu.acceleration_z_m_s2}]")

async def setup_offboard_control(drone):
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard mode error: {error}")

    print("-- Sending commands to drone")
    for _ in range(10):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.0, 0.0, 0.0, 0.5))
        await asyncio.sleep(1)
    
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Offboard mode error: {error}")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
