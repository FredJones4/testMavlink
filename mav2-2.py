import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError

async def send_commands(drone):
    throttle = 0.0
    roll = -1.0
    pitch = 1.0
    yaw = -1.0

    increment = 0.1

    while True:
        try:
            print(f"Sending throttle: {throttle}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")

            # Set the actuator values one by one
            await drone.action.set_actuator(1, throttle)  # Channel 1, RC_MAP_THROTTLE
            await drone.action.set_actuator(2, roll)      # Channel 2, RC_MAP_ROLL
            await drone.action.set_actuator(3, pitch)     # Channel 3, RC_MAP_PITCH
            await drone.action.set_actuator(4, yaw)       # Channel 4, RC_MAP_YAW
            await drone.action.set_actuator(5, 0.0)       # Channel 5 -- auxiliary
            await drone.action.set_actuator(6, 0.0)       # Channel 6 -- auxiliary, RC_MAP_FLTMODE
            await drone.action.set_actuator(7, 1.0)       # Channel 7 -- auxiliary, RC_MAP_OFFB_SW
            await drone.action.set_actuator(8, 0.0)       # Channel 8 -- auxiliary

            roll += increment
            throttle += increment
            pitch += increment
            yaw += increment

            if roll > 1.0 or throttle > 1.0:
                increment = 0  # Modify increment to 0 to stop changing values
            elif roll < 0.0 or throttle < 0.0:
                increment = 0  # Modify increment to 0 to stop changing values

            await asyncio.sleep(1)  # Adjust as needed
        except OffboardError as error:
            print(f"OffboardError: {error}")
            await asyncio.sleep(1)  # Wait before retrying
        except Exception as e:
            print(f"Unexpected error: {e}")
            await asyncio.sleep(1)

async def request_telemetry_data(drone):
    print("Requesting telemetry data...")
    async for position in drone.telemetry.position():
        print(f"Position: {position.latitude_deg}, {position.longitude_deg}, {position.absolute_altitude_m}")
        await asyncio.sleep(1)

    print("Requesting attitude data...")
    async for attitude in drone.telemetry.attitude_euler():
        print(f"Attitude: roll {attitude.roll_deg}, pitch {attitude.pitch_deg}, yaw {attitude.yaw_deg}")
        await asyncio.sleep(1)

    print("Requesting velocity data...")
    async for velocity in drone.telemetry.velocity_ned():
        print(f"Velocity NED: {velocity}")
        await asyncio.sleep(1)

    print("Requesting airspeed data...")
    async for airspeed in drone.telemetry.airspeed():
        print(f"Airspeed: {airspeed.airspeed_m_s}")
        await asyncio.sleep(1)

    print("Requesting RC status data...")
    async for rc_status in drone.telemetry.rc_status():
        print(f"RC Status: {rc_status}")
        await asyncio.sleep(1)

async def request_accelerometer_data(drone):
    print("Requesting accelerometer data...")
    async for imu in drone.telemetry.raw_imu():
        print("Raw IMU data attributes:")
        print(dir(imu))  # Print available attributes of the raw IMU object
        await asyncio.sleep(1)  # Adjust as needed

async def run():
    drone = System()
    # Connect to the Pixhawk via telemetry radio on /dev/ttyUSB0
    await drone.connect(system_address="serial:///dev/ttyUSB0:57600")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
        print("Offboard mode started successfully.")
    except OffboardError as e:
        print(f"Failed to start offboard mode: {e}")

    print("Starting tasks...")
    thrust_task = asyncio.ensure_future(send_commands(drone))
    telemetry_task = asyncio.ensure_future(request_telemetry_data(drone))
    accelerometer_task = asyncio.ensure_future(request_accelerometer_data(drone))

    await asyncio.gather(thrust_task, telemetry_task, accelerometer_task)

if __name__ == "__main__":
    asyncio.run(run())
