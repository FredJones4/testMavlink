import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, ActuatorControl

async def send_proof_of_life(drone):
    print("Sending proof of life signal")
    try:
        await drone.offboard.set_actuator_control(ActuatorControl([0.0] * 8))
    except OffboardError as error:
        print(f"Error: {error}")
    except Exception as e:
        print(f"Unexpected error: {e}")

async def send_command(drone):
    throttle = 0.5
    roll = 0.5
    pitch = 0.5
    yaw = 0.5

    try:
        print(f"Sending throttle: {throttle}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")
        await drone.offboard.set_actuator_control(
            ActuatorControl([
                throttle,   # Channel 1, RC_MAP_THROTTLE
                roll,       # Channel 2, RC_MAP_ROLL
                pitch,      # Channel 3, RC_MAP_PITCH
                yaw,        # Channel 4, RC_MAP_YAW
                0.0,        # Channel 5 -- auxiliary
                0.0,        # Channel 6 -- auxiliary, RC_MAP_FLTMODE
                1.0,        # Channel 7 -- auxiliary, RC_MAP_OFFB_SW
                0.0         # Channel 8 -- auxiliary
            ])
        )
    except OffboardError as error:
        print(f"Error: {error}")
    except Exception as e:
        print(f"Unexpected error: {e}")

async def request_sensor_data(drone):
    async for health in drone.telemetry.health():
        print(f"Health: {health}")
        break

async def run():
    drone = System()
    # Connect to the Pixhawk via telemetry radio on /dev/ttyUSB0
    await drone.connect(system_address="serial:///dev/ttyUSB0:57600")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    try:
        # Start offboard mode
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Error starting offboard mode: {error}")
        return

    await send_proof_of_life(drone)
    await send_command(drone)
    await request_sensor_data(drone)

if __name__ == "__main__":
    asyncio.run(run())
