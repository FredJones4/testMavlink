import time
from mavsdk import System
from mavsdk.offboard import OffboardError, ActuatorControl

# Function to send proof of life signal
def send_proof_of_life(drone):
    print("Sending proof of life signal")
    try:
        drone.offboard.set_actuator_control(ActuatorControl([0.0] * 8))
        time.sleep(1 / 2.5)
    except OffboardError as error:
        print(f"Offboard error: {error}")
    except Exception as e:
        print(f"Unexpected error: {e}")

# Function to request sensor data
def request_sensor_data(drone):
    try:
        health = drone.telemetry.health()
        print(f"Health: {health}")
    except Exception as e:
        print(f"Unexpected error: {e}")

# Function to send actuator control commands
def send_commands(drone):
    throttle = 0.5
    roll = 0.1
    pitch = 0.1
    yaw = 0.1

    try:
        print(f"Sending throttle: {throttle}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")
        drone.offboard.set_actuator_control(
            ActuatorControl([
                throttle,  # Channel 1, RC_MAP_THROTTLE
                roll,      # Channel 2, RC_MAP_ROLL
                pitch,     # Channel 3, RC_MAP_PITCH
                yaw,       # Channel 4, RC_MAP_YAW
                0.0,       # Channel 5 -- auxiliary
                0.0,       # Channel 6 -- auxiliary, RC_MAP_FLTMODE
                1.0,       # Channel 7 -- auxiliary, RC_MAP_OFFB_SW
                0.0        # Channel 8 -- auxiliary
            ])
        )
    except OffboardError as error:
        print(f"Offboard error: {error}")
    except Exception as e:
        print(f"Unexpected error: {e}")

# Main function to run the tasks sequentially
def run():
    drone = System()
    # Connect to the Pixhawk via telemetry radio on /dev/ttyUSB0
    drone.connect(system_address="serial:///dev/ttyUSB0:57600")

    print("Waiting for drone to connect...")
    while True:
        state = drone.core.connection_state()
        if state.is_connected:
            print("Drone connected!")
            break
        time.sleep(1)

    # Start offboard mode
    try:
        drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard start error: {error}")
        return

    send_proof_of_life(drone)
    send_commands(drone)
    request_sensor_data(drone)

    # Stop offboard mode
    try:
        drone.offboard.stop()
    except OffboardError as error:
        print(f"Offboard stop error: {error}")

if __name__ == "__main__":
    run()
