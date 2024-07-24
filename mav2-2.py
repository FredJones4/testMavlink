import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, ActuatorControl, ActuatorControlGroup, PositionNedYaw

# Define the rate at which the functions will run (200 Hz)
RATE_HZ = 200

async def send_commands(drone):
    throttle = 0.0
    roll = -1.0
    pitch = 1.0
    yaw = -1.0

    increment = 0.1

    while True:
        try:
            print(f"Sending throttle: {throttle}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")

            # Set the actuator values using offboard mode
            await drone.offboard.set_actuator_control(
                ActuatorControl(
                    [
                        ActuatorControlGroup(
                            [
                                throttle,  # Channel 1, RC_MAP_THROTTLE
                                roll,      # Channel 2, RC_MAP_ROLL
                                pitch,     # Channel 3, RC_MAP_PITCH
                                yaw,       # Channel 4, RC_MAP_YAW
                                0.0,       # Channel 5 -- auxiliary
                                0.0,       # Channel 6 -- auxiliary, RC_MAP_FLTMODE
                                1.0,       # Channel 7 -- auxiliary, RC_MAP_OFFB_SW
                                0.0        # Channel 8 -- auxiliary
                            ]
                        )
                    ]
                )
            )

            roll += increment
            throttle += increment
            pitch += increment
            yaw += increment

            if roll > 1.0 or throttle > 1.0:
                increment = 0  # Stop changing values
            elif roll < 0.0 or throttle < 0.0:
                increment = 0  # Stop changing values

            await asyncio.sleep(1 / RATE_HZ)  # Run at 200 Hz
        except OffboardError as error:
            print(f"OffboardError: {error}")
            await asyncio.sleep(1)  # Wait before retrying
        except Exception as e:
            print(f"Unexpected error: {e}")
            await asyncio.sleep(1)

async def request_telemetry_data(drone):
    print("Requesting sensor data continuously...")
    while True:
        print("Requesting telemetry data...")

        # Continuously retrieve and print position data
        async for position in drone.telemetry.position():
            print(f"Position: {position.latitude_deg}, {position.longitude_deg}, {position.absolute_altitude_m}")
            break

        # Continuously retrieve and print attitude data
        async for attitude in drone.telemetry.attitude_euler():
            print(f"Attitude: roll {attitude.roll_deg}, pitch {attitude.pitch_deg}, yaw {attitude.yaw_deg}")
            break

        # Continuously retrieve and print velocity data
        async for velocity in drone.telemetry.velocity_ned():
            print(f"Velocity NED: {velocity}")
            break

        # Continuously retrieve and print airspeed data
        async for airspeed in drone.telemetry.airspeed():
            print(f"Airspeed: {airspeed.airspeed_m_s}")
            break

        # Continuously retrieve and print RC status data
        async for rc_status in drone.telemetry.rc_status():
            print(f"RC Status: {rc_status}")
            break

        print("Requesting accelerometer data...")

        # Continuously retrieve and print raw IMU data
        async for imu in drone.telemetry.raw_imu():
            print("Raw IMU data:")
            print(f"Accelerometer (X, Y, Z): ({imu.accelerometer_m_s2[0]}, {imu.accelerometer_m_s2[1]}, {imu.accelerometer_m_s2[2]})")
            print(f"Gyroscope (X, Y, Z): ({imu.gyroscope_rad_s[0]}, {imu.gyroscope_rad_s[1]}, {imu.gyroscope_rad_s[2]})")
            print(f"Magnetometer (X, Y, Z): ({imu.magnetometer_ga[0]}, {imu.magnetometer_ga[1]}, {imu.magnetometer_ga[2]})")
            break

        await asyncio.sleep(1 / RATE_HZ)  # Run at 200 Hz

async def request_health_data(drone):
    async for health in drone.telemetry.health():
        print(f"Health: {health}")
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
    asyncio.sleep(1)
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break
    
    

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
        print("Offboard mode started successfully.")
    except OffboardError as e:
        print(f"Failed to start offboard mode: {e}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    print("-- Go 0m North, 0m East, -5m Down \
            within local coordinate system")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    await asyncio.sleep(10)

    print("Starting tasks...")
    thrust_task = asyncio.ensure_future(send_commands(drone))
    health_task = asyncio.ensure_future(request_health_data(drone))
    telemetry_task = asyncio.ensure_future(request_telemetry_data(drone))
    await asyncio.gather(thrust_task, health_task, telemetry_task)

    

if __name__ == "__main__":
    asyncio.run(run())
