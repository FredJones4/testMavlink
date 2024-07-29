"""
In the PX4 flight stack and MAVSDK, the specific mapping of actuator controls to roll, pitch, yaw, and thrust can vary based on the vehicle configuration (e.g., quadcopter, fixed-wing, etc.). However, for a standard multirotor (like a quadcopter), the actuator controls typically correspond to the following:

    Index 0: Roll control
    Index 1: Pitch control
    Index 2: Yaw control
    Index 3: Thrust control

For other types of vehicles, such as fixed-wing aircraft or tiltrotors, the mapping might differ, and additional indices might control other actuators like flaps, ailerons, etc.

Here’s a brief overview of what each index in the ActuatorControlGroup array typically controls for a multirotor:

    Index 0 (Roll): This controls the roll of the aircraft, which is the rotation around the front-to-back axis.
    Index 1 (Pitch): This controls the pitch of the aircraft, which is the rotation around the left-to-right axis.
    Index 2 (Yaw): This controls the yaw of the aircraft, which is the rotation around the vertical axis.
    Index 3 (Thrust): This controls the thrust, which directly affects the altitude of the aircraft.

The remaining indices (4 to 7) can be used for other actuator controls or remain unused (set to zero or NaN).

...
actuator_control_group = ActuatorControlGroup(
    controls=[0.0, 0.5, 0.0, 0.7, 0.0, 0.0, 0.0, 0.0]
)

actuator_controls = ActuatorControl(
    groups=[actuator_control_group]
)
...

In this example:

    0.0 for Roll (no roll movement)
    0.5 for Pitch (some pitch forward) [ or down, rather]
    0.0 for Yaw (no yaw movement)
    0.7 for Thrust (significant thrust)

This setup would cause the drone to pitch forward while maintaining a certain thrust level.
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, ActuatorControl, ActuatorControlGroup, PositionNedYaw

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

    print("-- Setting initial position setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Setting initial actuator control")
    # Assuming that we are controlling the first group (group 0)
    actuator_control_group = ActuatorControlGroup(
        controls=[0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )

    actuator_control_group_land = ActuatorControlGroup(
        controls=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    actuator_controls_land = ActuatorControl(
        groups=[actuator_control_group_land]
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

    print("-- Go 5m North, 0m East, -5m Down within local coordinate system, turn to face East")
    await drone.offboard.set_position_ned(PositionNedYaw(5.0, 0.0, -5.0, 90.0))
    await asyncio.sleep(10)

    print("-- Setting actuator control")
    await drone.offboard.set_actuator_control(actuator_controls)
    await asyncio.sleep(10)

    print("-- Setting actuator control to land")
    await drone.offboard.set_actuator_control(actuator_controls_land)
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
