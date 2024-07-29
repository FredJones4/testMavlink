import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, ActuatorControl, ActuatorControlGroup

async def run():
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

    print("-- Setting initial actuator control")
    
    # Assuming that we are controlling the first group (group 0)
    actuator_control_group = ActuatorControlGroup(
        controls=[0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    
    actuator_controls = ActuatorControl(
        groups=[actuator_control_group]
    )

    print("-- Setting initial actuator control setpoint")
    await drone.offboard.set_actuator_control(actuator_controls)

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Setting actuator control")
    await drone.offboard.set_actuator_control(actuator_controls)
    await asyncio.sleep(10)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Disarming")
    await drone.action.disarm()

if __name__ == "__main__":
    asyncio.run(run())
