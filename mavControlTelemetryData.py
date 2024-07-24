import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError

# Macro for proof of life frequency (Hz)
PROOF_OF_LIFE_HZ = 2.5

async def send_proof_of_life(drone):
    while True:
        print("Sending proof of life signal")
        # Set all actuators to 0 as a proof of life signal
        for i in range(1, 9):
            await drone.action.set_actuator(i, 0.0)
        await asyncio.sleep(1 / PROOF_OF_LIFE_HZ)

async def request_sensor_data(drone):
    async for health in drone.telemetry.health():
        print(f"Health: {health}")
        await asyncio.sleep(1)  # Adjust as needed

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
            # await drone.action.set_actuator(5, 0.0)       # Channel 5 -- auxiliary
            # await drone.action.set_actuator(6, 0.0)       # Channel 6 -- auxiliary, RC_MAP_FLTMODE
            # await drone.action.set_actuator(7, 1.0)       # Channel 7 -- auxiliary, RC_MAP_OFFB_SW
            # await drone.action.set_actuator(8, 0.0)       # Channel 8 -- auxiliary

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
    async for position in drone.telemetry.position():
        print(f"Position: {position}")
        await asyncio.sleep(1)

    async for attitude in drone.telemetry.attitude_euler():
        print(f"Attitude: {attitude}")
        await asyncio.sleep(1)

    async for velocity in drone.telemetry.velocity_ned():
        print(f"Velocity NED: {velocity}")
        await asyncio.sleep(1)

    async for airspeed in drone.telemetry.airspeed():
        print(f"Airspeed: {airspeed}")
        await asyncio.sleep(1)

    async for rc_status in drone.telemetry.rc_status():
        print(f"RC Status: {rc_status}")
        await asyncio.sleep(1)

async def request_accelerometer_data(drone):
    async for imu in drone.telemetry.raw_imu():
        print(f"Accelerometer: {imu.acceleration_frd}")
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

    # Start offboard mode if required
    # await drone.offboard.start()

    control_task = asyncio.ensure_future(send_proof_of_life(drone))
    sensor_task = asyncio.ensure_future(request_sensor_data(drone))
    thrust_task = asyncio.ensure_future(send_commands(drone))
    telemetry_task = asyncio.ensure_future(request_telemetry_data(drone))
    accelerometer_task = asyncio.ensure_future(request_accelerometer_data(drone))

    await asyncio.gather(control_task, sensor_task, thrust_task, telemetry_task, accelerometer_task)

if __name__ == "__main__":
    asyncio.run(run())
