import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, ActuatorControl, ActuatorControlGroup

# Macro for proof of life frequency (Hz)
PROOF_OF_LIFE_HZ = 2.5

async def send_proof_of_life(drone):
	while True:
		print("Sending proof of life signal")
		# Replace this with an actual "proof of life" command if available
		await drone.offboard.set_actuator_control(
			ActuatorControl(
					[
					ActuatorControlGroup([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
					]
				)
			)
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

	increment =  0.1
	
	while True:
		try:
			# Thrust and throttle values (varying from 0 to 1)
			print(f"Sending throttle: {throttle}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")
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
                        ])
					# 	,
                                                
                    # ActuatorControlGroup([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
					]
					)
				)
			
			roll += increment
			throttle += increment
			pitch += increment
			yaw += increment
			
			if roll > 1.0 or throttle > 1.0: # was 1.0 #TODO: modify for testing individual channels
				increment = 0#-0.1
			elif roll < 0.0 or throttle < 0.0:
				increment = 0#0.1
			
			await asyncio.sleep(1)  # Adjust as needed
		except OffboardError as error:
			print(f"Error: {error}")
			await asyncio.sleep(1)  # Wait before retrying
		except Exception as e:
			print(f"Unexpected error: {e}")
			await asyncio.sleep(1)

async def run():
	drone = System()
	# Connect to the Pixhawk via telemetry radio on /dev/ttyUSB0
	await drone.connect(system_address="serial:///dev/ttyUSB0:57600")

	print("Waiting for drone to connect...")
	async for state in drone.core.connection_state():
		if state.is_connected:
			print("Drone connected!")
			break

	# 	    # Start offboard mode
    # await drone.offboard.start()

	control_task = asyncio.ensure_future(send_proof_of_life(drone))
	sensor_task = asyncio.ensure_future(request_sensor_data(drone))
	thrust_task = asyncio.ensure_future(send_commands(drone))

	await asyncio.gather(control_task, sensor_task, thrust_task)

if __name__ == "__main__":
	asyncio.run(run())
