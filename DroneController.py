"""
To run this:
Must instantiate DroneController with a System object and call the methods as needed.

See test cases of inidividual methods in 1_and_3_dean_new_connect_receive_data.py and 2_1_test_both_control.py.

For future reference, a means to calculate alpha and beta is found in 1-1_dean_alpha_beta.py.
"""
import numpy as np
import asyncio
from mavsdk import System
from mavsdk.offboard import Offboard, OffboardError, ActuatorControl, ActuatorControlGroup, PositionNedYaw
from mavsdk.action import Action
from mavsdk.telemetry import Telemetry

CURRENT_CONNECTION = "udp://:14540"

class DroneController:
    def __init__(self, drone: System):
        self.drone = drone

    @staticmethod
    def clamp(value, min_value, max_value):
        """Clamps the value between min_value and max_value."""
        return max(min_value, min(value, max_value))

    async def set_actuator_control_homemade(self, elevator, aileron, rudder, starboard_front_motor, 
                                            rear_motor, front_motor, starboard_rear_motor, forward_propulsion_motor):
        """Sets the actuator control with two control groups and specific control modifications."""
        group1_controls = [
            float('nan'),  # NaN for RC ROLL
            float('nan'),  # NaN for RC PITCH
            float('nan'),  # NaN for RC YAW
            float('nan'),  # NaN for RC Throttle
            self.clamp(elevator, -1, 1),
            self.clamp(aileron, -1, 1),  # Left aileron
            self.clamp(rudder, -1, 1),
            self.clamp(-aileron, -1, 1)  # Reversed aileron for right 
        ]

        group2_controls = [
            float('nan'),  # Unused
            float('nan'),  # Unused
            float('nan'),  # Unused
            self.clamp(starboard_front_motor, 0, 1), 
            self.clamp(rear_motor, 0, 1),
            self.clamp(front_motor, 0, 1),
            self.clamp(starboard_rear_motor, 0, 1),
            self.clamp(forward_propulsion_motor, 0, 1)
        ]
        
        control_group1 = ActuatorControlGroup(controls=group1_controls)
        control_group2 = ActuatorControlGroup(controls=group2_controls)

        try:
            await self.drone.offboard.set_actuator_control(ActuatorControl(groups=[control_group1, control_group2]))
        except OffboardError as e:
            print(f"Setting actuator control failed with error: {e}")

    @staticmethod
    def print_pretty_dict(d, indent=4):
        """Prints the contents of a dictionary (and nested dictionaries) in a pretty format."""
        def print_dict(d, level=0):
            for key, value in d.items():
                if isinstance(value, dict):
                    print(' ' * (level * indent) + f"{key}:")
                    print_dict(value, level + 1)
                else:
                    print(' ' * (level * indent) + f"{key}: {value}")

        print_dict(d)

    async def setup_mavlink_offboard(self, curr_conn=CURRENT_CONNECTION):
        """Sets up the MAVSDK MAVLink protocols."""
        await self.drone.connect(system_address=curr_conn)

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

        print("-- Arming")
        await self.drone.action.arm()

        print("-- Setting initial setpoint")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")
            print("-- Disarming")
            await self.drone.action.disarm()
            return False

        return True

    @staticmethod
    def euler_to_rotation_matrix(roll, pitch, yaw):
        """Convert Euler angles to a rotation matrix."""
        roll_rad = np.radians(roll)
        pitch_rad = np.radians(pitch)
        yaw_rad = np.radians(yaw)

        R_z = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
            [np.sin(yaw_rad), np.cos(yaw_rad), 0],
            [0, 0, 1]
        ])

        R_y = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
        ])

        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad), np.cos(roll_rad)]
        ])

        R = R_z @ R_y @ R_x
        return R

    @staticmethod
    def calculate_velocity_body(velocity_ned, euler_angles):
        """Calculate body frame velocities from NED frame velocities and Euler angles."""
        R = DroneController.euler_to_rotation_matrix(
            euler_angles['roll_deg'],
            euler_angles['pitch_deg'],
            euler_angles['yaw_deg']
        )

        ned_velocity = np.array([
            velocity_ned['north_m_s'],
            velocity_ned['east_m_s'],
            velocity_ned['down_m_s']
        ])

        body_velocity = R @ ned_velocity

        return {
            'body_x_m_s': body_velocity[0],
            'body_y_m_s': body_velocity[1],
            'body_z_m_s': body_velocity[2]
        }

    async def collect_telemetry_data(self):
        """Collect telemetry data from the drone.
        TODO: See if it is worth it to hold old data values from last run so that, if a value is not grabbed, it will hold the previous value.
        """
        data = {}

        async def request_rc_status():
            try:
                if not hasattr(self.drone.telemetry, 'rc_status'):
                    data['rc_status_error'] = "RC Status method not available."
                    return

                rc_status_iter = self.drone.telemetry.rc_status()
                try:
                    rc_status = await asyncio.wait_for(rc_status_iter.__anext__(), timeout=0.1)
                    data['rc_status'] = {
                        'is_available': rc_status.is_available,
                        'was_available_once': rc_status.was_available_once,
                        'signal_strength_percent': rc_status.signal_strength_percent
                    }
                except asyncio.TimeoutError:
                    data['rc_status'] = {
                        'is_available': 0,
                        'was_available_once': 0,
                        'signal_strength_percent': 0
                    }
            except AttributeError as e:
                data['rc_status_error'] = str(e)

        async def request_raw_imu():
            try:
                if not hasattr(self.drone.telemetry, 'raw_imu'):
                    data['raw_imu_error'] = "Raw IMU method not available."
                    return

                raw_imu_iter = self.drone.telemetry.raw_imu()
                try:
                    imu = await asyncio.wait_for(raw_imu_iter.__anext__(), timeout=0.1)
                    data['raw_imu'] = {
                        'acceleration_frd': imu.acceleration_frd,
                        'gyroscope_frd': imu.gyroscope_frd,
                        'magnetometer_frd': imu.magnetometer_frd
                    }
                except asyncio.TimeoutError:
                    data['raw_imu'] = {
                        'acceleration_frd': 0,
                        'gyroscope_frd': 0,
                        'magnetometer_frd': 0
                    }
            except AttributeError as e:
                data['raw_imu_error'] = str(e)

        async def request_euler_angles():
            try:
                euler_angles_iter = self.drone.telemetry.attitude_euler()
                try:
                    euler_angle = await asyncio.wait_for(euler_angles_iter.__anext__(), timeout=0.1)
                    data['euler_angles'] = {
                        'roll_deg': euler_angle.roll_deg,
                        'pitch_deg': euler_angle.pitch_deg,
                        'yaw_deg': euler_angle.yaw_deg,
                        'timestamp_us': euler_angle.timestamp_us
                    }
                except asyncio.TimeoutError:
                    data['euler_angles'] = {
                        'roll_deg': 0,
                        'pitch_deg': 0,
                        'yaw_deg': 0,
                        'timestamp_us': 0
                    }
            except AttributeError:
                data['euler_angles_error'] = "Euler Angle method not available."

        async def request_angular_velocity():
            try:
                if not hasattr(self.drone.telemetry, 'attitude_angular_velocity_body'):
                    data['angular_velocity_error'] = "Attitude Angular Velocity Body method not available."
                    return

                angular_velocity_iter = self.drone.telemetry.attitude_angular_velocity_body()
                try:
                    angular_velocity = await asyncio.wait_for(angular_velocity_iter.__anext__(), timeout=0.1)
                    data['angular_velocity'] = {
                        'roll_rad_s': angular_velocity.roll_rad_s,
                        'pitch_rad_s': angular_velocity.pitch_rad_s,
                        'yaw_rad_s': angular_velocity.yaw_rad_s
                    }
                except asyncio.TimeoutError:
                    data['angular_velocity'] = {
                        'roll_rad_s': 0,
                        'pitch_rad_s': 0,
                        'yaw_rad_s': 0
                    }
            except AttributeError as e:
                data['angular_velocity_error'] = str(e)

        async def request_airspeed():
            try:
                airspeed_iter = self.drone.telemetry.fixedwing_metrics()
                try:
                    metrics = await asyncio.wait_for(airspeed_iter.__anext__(), timeout=0.1)
                    data['airspeed'] = {
                        'airspeed_m_s': metrics.airspeed_m_s,
                        'throttle_percentage': metrics.throttle_percentage,
                        'climb_rate_m_s': metrics.climb_rate_m_s
                    }
                except asyncio.TimeoutError:
                    data['airspeed'] = {
                        'airspeed_m_s': 0,
                        'throttle_percentage': 0,
                        'climb_rate_m_s': 0
                    }
            except AttributeError:
                data['airspeed_error'] = "FixedwingMetrics method not available."

        async def request_position():
            try:
                position_iter = self.drone.telemetry.position()
                try:
                    position = await asyncio.wait_for(position_iter.__anext__(), timeout=0.1)
                    data['position'] = {
                        'latitude_deg': position.latitude_deg,
                        'longitude_deg': position.longitude_deg,
                        'absolute_altitude_m': position.absolute_altitude_m,
                        'relative_altitude_m': position.relative_altitude_m
                    }
                except asyncio.TimeoutError:
                    data['position'] = {
                        'latitude_deg': 0,
                        'longitude_deg': 0,
                        'absolute_altitude_m': 0,
                        'relative_altitude_m': 0
                    }
            except AttributeError:
                data['position_error'] = "Position method not available."

        async def request_velocity_ned():
            try:
                velocity_ned_iter = self.drone.telemetry.velocity_ned()
                try:
                    velocity = await asyncio.wait_for(velocity_ned_iter.__anext__(), timeout=0.1)
                    data['velocity_ned'] = {
                        'north_m_s': velocity.north_m_s,
                        'east_m_s': velocity.east_m_s,
                        'down_m_s': velocity.down_m_s
                    }
                except asyncio.TimeoutError:
                    data['velocity_ned'] = {
                        'north_m_s': 0,
                        'east_m_s': 0,
                        'down_m_s': 0
                    }
            except AttributeError:
                data['velocity_ned_error'] = "Velocity NED method not available."

        # Run all the requests concurrently
        await asyncio.gather(
            request_rc_status(),
            request_raw_imu(),
            request_euler_angles(),
            request_angular_velocity(),
            request_airspeed(),
            request_position(),
            request_velocity_ned()
        )

        if 'velocity_ned' in data and 'euler_angles' in data:
            data['velocity_body'] = self.calculate_velocity_body(data['velocity_ned'], data['euler_angles'])
        else:
            data['velocity_body'] = {
                'body_x_m_s': 0,
                'body_y_m_s': 0,
                'body_z_m_s': 0
            }

        return data
