import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

async def send_control_commands(drone):
	while True:
		try:
			# Example of sending a simple control command
			# You should replace PositionNedYaw with your desired command type and parameters
			await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))
			await asyncio.sleep(1)  # Adjust as needed
		except OffboardError as error:
			print(f"Error: {error}")
			await asyncio.sleep(1)  # Wait before retrying
		except Exception as e:
			print(f"Unexpected error: {e}")
			await asyncio.sleep(1)

async def request_sensor_data(drone):
	async for health in drone.telemetry.health():
		print(f"Health: {health}")
		await asyncio.sleep(1)  # Adjust as needed

async def run():
	drone = System()
	await drone.connect(system_address="udp://:14540")  # Adjust the connection string as needed

	print("Waiting for drone to connect...")
	async for state in drone.core.connection_state():
		if state.is_connected:
			print("Drone connected!")
			break

	print("Waiting for drone to have a global position estimate...")
	async for health in drone.telemetry.health():
		if health.is_global_position_ok:
			print("Global position estimate OK")
			break

	print("Arming the drone...")
	await drone.action.arm()

	print("Starting offboard mode...")
	try:
		await drone.offboard.start()
	except OffboardError as error:
		print(f"Starting offboard mode failed with error code: {error}")
		print("-- Disarming")
		await drone.action.disarm()
		return

	control_task = asyncio.ensure_future(send_control_commands(drone))
	sensor_task = asyncio.ensure_future(request_sensor_data(drone))

	await asyncio.gather(control_task, sensor_task)

if __name__ == "__main__":
	asyncio.run(run())
