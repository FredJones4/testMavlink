import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, ActuatorControl

# Macro for proof of life frequency (Hz)
PROOF_OF_LIFE_HZ = 2.5

async def send_proof_of_life(drone):
    while True:
        print("Sending proof of life signal")
        # Replace this with an actual "proof of life" command if available
        await drone.offboard.set_actuator_control(ActuatorControl([0.0] * 8))
        await asyncio.sleep(1 / PROOF_OF_LIFE_HZ)

async def request_sensor_data(drone):
    async for health in drone.telemetry.health():
        print(f"Health: {health}")
        await asyncio.sleep(1)  # Adjust as needed

async def send_thrust_and_throttle(drone):
    thrust = 0.0
    throttle = 0.0
    increment = 0.1
    
    while True:
        try:
            # Thrust and throttle values (varying from 0 to 1)
            print(f"Sending thrust: {thrust}, throttle: {throttle}")
            await drone.offboard.set_actuator_control(
                ActuatorControl([
                    thrust,    # Thrust
                    throttle,  # Throttle
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                ])
            )
            
            thrust += increment
            throttle += increment
            
            if thrust > 1.0 or throttle > 1.0:
                increment = -0.1
            elif thrust < 0.0 or throttle < 0.0:
                increment = 0.1
            
            await asyncio.sleep(1)  # Adjust as needed
        except OffboardError as error:
            print(f"Error: {error}")
            await asyncio.sleep(1)  # Wait before retrying
        except Exception as e:
            print(f"Unexpected error: {e}")
            await asyncio.sleep(1)

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")  # Adjust the connection string as needed

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    control_task = asyncio.ensure_future(send_proof_of_life(drone))
    sensor_task = asyncio.ensure_future(request_sensor_data(drone))
    thrust_task = asyncio.ensure_future(send_thrust_and_throttle(drone))

    await asyncio.gather(control_task, sensor_task, thrust_task)

if __name__ == "__main__":
    asyncio.run(run())
