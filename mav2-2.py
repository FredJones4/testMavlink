import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError

# Macro for proof of life frequency (Hz)
PROOF_OF_LIFE_HZ = 2.5

async def send_proof_of_life(drone):
    while True:
        print("Sending proof of life signal")
        for i in range(1, 9):
            await drone.action.set_actuator(i, 0.0)
        await asyncio.sleep(1 / PROOF_OF_LIFE_HZ)

async def send_commands(drone):
    throttle = 0.0
    roll = -1.0
    pitch = 1.0
    yaw = -1.0
    increment = 0.1

    while True:
        try:
            print(f"Sending throttle: {throttle}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")
            await drone.action.set_actuator(1, throttle)
            await drone.action.set_actuator(2, roll)
            await drone.action.set_actuator(3, pitch)
            await drone.action.set_actuator(4, yaw)
            # await drone.action.set_actuator(5, 0.0)
            # await drone.action.set_actuator(6, 0.0)
            # await drone.action.set_actuator(7, 1.0)
            # await drone.action.set_actuator(8, 0.0)

            roll += increment
            throttle += increment
            pitch += increment
            yaw += increment

            if roll > 1.0 or throttle > 1.0:
                increment = 0
            elif roll < 0.0 or throttle < 0.0:
                increment = 0

            await asyncio.sleep(1)
        except OffboardError as error:
            print(f"OffboardError: {error}")
            await asyncio.sleep(1)
        except Exception as e:
            print(f"Unexpected error: {e}")
            await asyncio.sleep(1)

async def request_acceleration_frd(drone):
    async for acceleration in drone.telemetry.AccelerationFrd():
        print(f"AccelerationFrd: {acceleration}")
        await asyncio.sleep(1)

async def request_actuator_control_target(drone):
    async for target in drone.telemetry.ActuatorControlTarget():
        print(f"ActuatorControlTarget: {target}")
        await asyncio.sleep(1)

async def request_actuator_output_status(drone):
    async for status in drone.telemetry.ActuatorOutputStatus():
        print(f"ActuatorOutputStatus: {status}")
        await asyncio.sleep(1)

async def request_angular_velocity_body(drone):
    async for velocity in drone.telemetry.AngularVelocityBody():
        print(f"AngularVelocityBody: {velocity}")
        await asyncio.sleep(1)

async def request_angular_velocity_frd(drone):
    async for velocity in drone.telemetry.AngularVelocityFrd():
        print(f"AngularVelocityFrd: {velocity}")
        await asyncio.sleep(1)

async def request_battery(drone):
    async for battery in drone.telemetry.Battery():
        print(f"Battery: {battery}")
        await asyncio.sleep(1)

async def request_euler_angle(drone):
    async for angle in drone.telemetry.EulerAngle():
        print(f"EulerAngle: {angle}")
        await asyncio.sleep(1)

async def request_heading(drone):
    async for heading in drone.telemetry.Heading():
        print(f"Heading: {heading}")
        await asyncio.sleep(1)

async def request_odometry(drone):
    async for odometry in drone.telemetry.Odometry():
        print(f"Odometry: {odometry}")
        await asyncio.sleep(1)

async def request_position(drone):
    async for position in drone.telemetry.Position():
        print(f"Position: {position}")
        await asyncio.sleep(1)

async def request_position_body(drone):
    async for position in drone.telemetry.PositionBody():
        print(f"PositionBody: {position}")
        await asyncio.sleep(1)

async def request_position_ned(drone):
    async for position in drone.telemetry.PositionVelocityNed():
        print(f"PositionNed: {position}")
        await asyncio.sleep(1)

async def request_quaternion(drone):
    async for quaternion in drone.telemetry.Quaternion():
        print(f"Quaternion: {quaternion}")
        await asyncio.sleep(1)

async def request_raw_gps(drone):
    async for gps in drone.telemetry.RawGps():
        print(f"RawGps: {gps}")
        await asyncio.sleep(1)

async def request_status_text(drone):
    async for status_text in drone.telemetry.StatusText():
        print(f"StatusText: {status_text}")
        await asyncio.sleep(1)

async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyUSB0:57600")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    control_task = asyncio.ensure_future(send_proof_of_life(drone))
    thrust_task = asyncio.ensure_future(send_commands(drone))
    accel_frd_task = asyncio.ensure_future(request_acceleration_frd(drone))
    actuator_control_task = asyncio.ensure_future(request_actuator_control_target(drone))
    actuator_output_task = asyncio.ensure_future(request_actuator_output_status(drone))
    angular_velocity_body_task = asyncio.ensure_future(request_angular_velocity_body(drone))
    angular_velocity_frd_task = asyncio.ensure_future(request_angular_velocity_frd(drone))
    battery_task = asyncio.ensure_future(request_battery(drone))
    euler_angle_task = asyncio.ensure_future(request_euler_angle(drone))
    heading_task = asyncio.ensure_future(request_heading(drone))
    odometry_task = asyncio.ensure_future(request_odometry(drone))
    position_task = asyncio.ensure_future(request_position(drone))
    position_body_task = asyncio.ensure_future(request_position_body(drone))
    position_ned_task = asyncio.ensure_future(request_position_ned(drone))
    quaternion_task = asyncio.ensure_future(request_quaternion(drone))
    raw_gps_task = asyncio.ensure_future(request_raw_gps(drone))
    status_text_task = asyncio.ensure_future(request_status_text(drone))

    await asyncio.gather(
        control_task, thrust_task, accel_frd_task, actuator_control_task, actuator_output_task,
        angular_velocity_body_task, angular_velocity_frd_task, battery_task, euler_angle_task,
        heading_task, odometry_task, position_task, position_body_task, position_ned_task,
        quaternion_task, raw_gps_task, status_text_task
    )

if __name__ == "__main__":
    asyncio.run(run())
