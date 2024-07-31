#include <iostream>
#include <chrono>
#include <thread>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using namespace std::chrono;
using namespace std::this_thread;

void print_pretty_map(const std::map<std::string, std::string>& data, int indent = 4) {
    for (const auto& pair : data) {
        std::cout << std::string(indent, ' ') << pair.first << ": " << pair.second << std::endl;
    }
}

void calculate_velocity_body(const Telemetry::VelocityNed& velocity_ned, const Telemetry::EulerAngle& euler_angles, Telemetry::VelocityBody& velocity_body) {
    // Convert euler angles to rotation matrix
    double roll_rad = euler_angles.roll_deg * M_PI / 180.0;
    double pitch_rad = euler_angles.pitch_deg * M_PI / 180.0;
    double yaw_rad = euler_angles.yaw_deg * M_PI / 180.0;

    // Rotation matrices
    double R_z[3][3] = {
        {cos(yaw_rad), -sin(yaw_rad), 0},
        {sin(yaw_rad), cos(yaw_rad), 0},
        {0, 0, 1}
    };
    
    double R_y[3][3] = {
        {cos(pitch_rad), 0, sin(pitch_rad)},
        {0, 1, 0},
        {-sin(pitch_rad), 0, cos(pitch_rad)}
    };
    
    double R_x[3][3] = {
        {1, 0, 0},
        {0, cos(roll_rad), -sin(roll_rad)},
        {0, sin(roll_rad), cos(roll_rad)}
    };
    
    double R[3][3];
    // Combine rotation matrices
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = R_z[i][0] * R_y[0][j] + R_z[i][1] * R_y[1][j] + R_z[i][2] * R_y[2][j];
        }
    }

    double velocity_ned_arr[3] = {
        velocity_ned.north_m_s,
        velocity_ned.east_m_s,
        -velocity_ned.down_m_s
    };

    // Compute velocity body
    velocity_body.body_x_m_s = R[0][0] * velocity_ned_arr[0] + R[0][1] * velocity_ned_arr[1] + R[0][2] * velocity_ned_arr[2];
    velocity_body.body_y_m_s = R[1][0] * velocity_ned_arr[0] + R[1][1] * velocity_ned_arr[1] + R[1][2] * velocity_ned_arr[2];
    velocity_body.body_z_m_s = R[2][0] * velocity_ned_arr[0] + R[2][1] * velocity_ned_arr[1] + R[2][2] * velocity_ned_arr[2];
}

void collect_telemetry_data(Telemetry& telemetry) {
    auto odometry = telemetry.odometry();
    auto imu = telemetry.imu();

    // Collect telemetry data
    std::map<std::string, std::string> data;
    data["odometry_position_body_x_m"] = std::to_string(odometry.position_body().x_m);
    data["odometry_position_body_y_m"] = std::to_string(odometry.position_body().y_m);
    data["odometry_position_body_z_m"] = std::to_string(odometry.position_body().z_m);
    data["odometry_velocity_body_x_m_s"] = std::to_string(odometry.velocity_body().x_m_s);
    data["odometry_velocity_body_y_m_s"] = std::to_string(odometry.velocity_body().y_m_s);
    data["odometry_velocity_body_z_m_s"] = std::to_string(odometry.velocity_body().z_m_s);
    data["imu_acceleration_frd_x"] = std::to_string(imu.acceleration_frd().x);
    data["imu_acceleration_frd_y"] = std::to_string(imu.acceleration_frd().y);
    data["imu_acceleration_frd_z"] = std::to_string(imu.acceleration_frd().z);

    print_pretty_map(data);
}

int main() {
    Mavsdk mavsdk;
    System& drone = mavsdk.system();

    // Connect to the drone
    std::string connection_url = "udp://:14540";
    mavsdk.add_any_connection(connection_url);
    std::this_thread::sleep_for(seconds(1));

    if (!drone.is_connected()) {
        std::cerr << "Failed to connect to the drone!" << std::endl;
        return 1;
    }

    std::cout << "Connected to drone!" << std::endl;

    // Arm the drone
    Action action(drone);
    action.arm().wait();

    // Set initial setpoint and start offboard
    Offboard offboard(drone);
    offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0)).wait();
    offboard.start().wait();

    std::cout << "Offboard started, moving to initial position" << std::endl;

    // Move to initial positions
    offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0)).wait();
    std::this_thread::sleep_for(seconds(10));

    collect_telemetry_data(drone.telemetry());

    // Stop offboard and disarm the drone
    offboard.stop().wait();
    action.disarm().wait();

    std::cout << "Offboard stopped and drone disarmed" << std::endl;

    return 0;
}
