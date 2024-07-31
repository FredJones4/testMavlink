import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import numpy as np

# Import your DroneController class here
from DroneController import DroneController  # Replace 'your_module' with the actual module name

# Test the DroneController class
async def test_drone_controller():
    drone = System()
    controller = DroneController(drone)

    # Set up MAVLink and offboard mode
    success = await controller.setup_mavlink_offboard()
    if not success:
        print("Failed to set up MAVLink offboard mode.")
        return

    # Perform movements
    print("-- Go 0m North, 0m East, -5m Down within local coordinate system")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    await asyncio.sleep(10)

    # Collect telemetry data
    data = await controller.collect_telemetry_data()
    print("Collected telemetry data:")
    controller.print_pretty_dict(data)

    # Check velocity body calculation
    if 'velocity_ned' in data and 'euler_angles' in data:
        velocity_body = controller.calculate_velocity_body(data['velocity_ned'], data['euler_angles'])
        print("Calculated velocity body:")
        controller.print_pretty_dict(velocity_body)

    # Perform more movements
    print("-- Go 5m North, 0m East, -5m Down within local coordinate system, turn to face East")
    await drone.offboard.set_position_ned(PositionNedYaw(5.0, 0.0, -5.0, 90.0))
    await asyncio.sleep(10)

    print("-- Go 5m North, 10m East, -5m Down within local coordinate system")
    await drone.offboard.set_position_ned(PositionNedYaw(5.0, 10.0, -5.0, 90.0))
    await asyncio.sleep(15)

    print("-- Go 0m North, 10m East, 0m Down within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 10.0, 0.0, 180.0))
    await asyncio.sleep(10)

    # Stop offboard mode
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    # Collect telemetry data again
    data = await controller.collect_telemetry_data()
    print("Collected telemetry data after stopping offboard:")
    controller.print_pretty_dict(data)

# Run the test
if __name__ == "__main__":
    asyncio.run(test_drone_controller())
