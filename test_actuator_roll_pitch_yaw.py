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
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(up_air_position)

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
        await drone.offboard.set_actuator_control(actuator_controls)
        await asyncio.sleep(SLEEP_MID)

    # Test Roll control
    print("-- Testing Roll control")
    roll_control = ActuatorControlGroup(controls=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    await set_and_wait(ActuatorControl(groups=[roll_control]))

    # Reset to initial setpoint
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(SLEEP_MID)
    await drone.offboard.set_position_ned(up_air_position)
    await asyncio.sleep(SLEEP_MID)

    # Test Pitch control
    print("-- Testing Pitch control")
    pitch_control = ActuatorControlGroup(controls=[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    await set_and_wait(ActuatorControl(groups=[pitch_control]))

    # Reset to initial setpoint
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(SLEEP_MID)
    await drone.offboard.set_position_ned(up_air_position)
    await asyncio.sleep(SLEEP_MID)

    # Test Yaw control
    print("-- Testing Yaw control")
    yaw_control = ActuatorControlGroup(controls=[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    await set_and_wait(ActuatorControl(groups=[yaw_control]))

    # Reset to initial setpoint
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(SLEEP_MID)
    await drone.offboard.set_position_ned(up_air_position)
    await asyncio.sleep(SLEEP_MID)

    # Test Thrust control
    print("-- Testing Thrust control")
    thrust_control = ActuatorControlGroup(controls=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
    await set_and_wait(ActuatorControl(groups=[thrust_control]))

    # Reset to initial setpoint
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(SLEEP_MID)


    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Disarming")
    await drone.action.disarm()

if __name__ == "__main__":
    asyncio.run(run())


"""
Result:
Init MAVLink
INFO  [lockstep_scheduler] setting initial absolute time to 1722266325321000 us
INFO  [commander] LED: open /dev/led0 failed (22)
WARN  [health_and_arming_checks] Preflight Fail: ekf2 missing data
INFO  [uxrce_dds_client] init UDP agent IP:127.0.0.1, port:8888
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14280 remote port 14030
INFO  [mavlink] mode: Gimbal, data rate: 400000 B/s on udp port 13030 remote port 13280
INFO  [logger] logger started (mode=all)
INFO  [logger] Start file log (type: full)
INFO  [logger] [logger] ./log/2024-07-29/15_18_46.ulg	
INFO  [logger] Opened full log file: ./log/2024-07-29/15_18_46.ulg
INFO  [mavlink] MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)
INFO  [mavlink] MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)
INFO  [px4] Startup script returned successfully
pxh> INFO  [mavlink] partner IP: 127.0.0.1
WARN  [health_and_arming_checks] Preflight: GPS fix too low
INFO  [tone_alarm] home set
WARN  [health_and_arming_checks] Preflight: GPS fix too low
INFO  [commander] Ready for takeoff!
INFO  [commander] Armed by external command	
INFO  [tone_alarm] arming warning
WARN  [failsafe] Failsafe activated	
INFO  [tone_alarm] battery warning (fast)
ERROR [flight_mode_manager] Matching flight task was not able to run, Nav state: 2, Task: 1
INFO  [commander] Disarmed by auto preflight disarming	
INFO  [tone_alarm] notify neutral
INFO  [logger] closed logfile, bytes written: 8115936

"""