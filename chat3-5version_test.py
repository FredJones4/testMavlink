import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, ActuatorControl, ActuatorControlGroup, PositionNedYaw

SLEEP_SHORT = 2
SLEEP_MID = 5

async def run():
    up_air_position = PositionNedYaw(5.0, 0.0, -5.0, 90.0)
    ground_position = PositionNedYaw(0.0, 0.0, 0.0, 0.0)

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
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"Arming failed with error: {e}")
        return

    print("-- Setting initial setpoint")
    try:
        await drone.offboard.set_position_ned(up_air_position)
    except OffboardError as e:
        print(f"Setting initial setpoint failed with error: {e}")
        await drone.action.disarm()
        return

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Function to set actuator control and wait
    async def set_and_wait(actuator_controls):
        try:
            await drone.offboard.set_actuator_control(actuator_controls)
        except OffboardError as e:
            print(f"Setting actuator control failed with error: {e}")
        await asyncio.sleep(SLEEP_MID)

    # Test Roll control
    print("-- Testing Roll control")
    roll_control = ActuatorControlGroup(controls=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    await set_and_wait(ActuatorControl(groups=[roll_control]))

    # Reset to initial setpoint
    await drone.offboard.set_position_ned(ground_position)
    await asyncio.sleep(SLEEP_SHORT)
    await drone.offboard.set_position_ned(up_air_position)
    await asyncio.sleep(SLEEP_SHORT)

    # Test Pitch control
    print("-- Testing Pitch control")
    pitch_control = ActuatorControlGroup(controls=[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    await set_and_wait(ActuatorControl(groups=[pitch_control]))

    # Reset to initial setpoint
    await drone.offboard.set_position_ned(ground_position)
    await asyncio.sleep(SLEEP_SHORT)
    await drone.offboard.set_position_ned(up_air_position)
    await asyncio.sleep(SLEEP_SHORT)

    # Test Yaw control
    print("-- Testing Yaw control")
    yaw_control = ActuatorControlGroup(controls=[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    await set_and_wait(ActuatorControl(groups=[yaw_control]))

    # Reset to initial setpoint
    await drone.offboard.set_position_ned(ground_position)
    await asyncio.sleep(SLEEP_SHORT)
    await drone.offboard.set_position_ned(up_air_position)
    await asyncio.sleep(SLEEP_SHORT)

    # Test Thrust control
    print("-- Testing Thrust control")
    thrust_control = ActuatorControlGroup(controls=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
    await set_and_wait(ActuatorControl(groups=[thrust_control]))

    # Reset to initial setpoint
    await drone.offboard.set_position_ned(ground_position)
    await asyncio.sleep(SLEEP_SHORT)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Disarming")
    try:
        await drone.action.disarm()
    except Exception as e:
        print(f"Disarming failed with error: {e}")

if __name__ == "__main__":
    asyncio.run(run())
