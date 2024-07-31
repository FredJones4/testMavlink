#include <iostream>
#include <chrono>
#include <thread>
#include <map>  // Include this for std::map
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using namespace std::chrono;
using namespace std::this_thread;

// Function to print a map in a pretty format
void print_pretty_map(const std::map<std::string, std::string>& data, int indent = 4) {
    for (const auto& pair : data) {
        std::cout << std::string(indent, ' ') << pair.first << ": " << pair.second << std::endl;
    }
}

// Function to calculate velocity in the body frame
void calculate_velocity_body(const Telemetry::VelocityNed& velocity_ned,
                              const Telemetry::EulerAngle& euler_angle,
                              Telemetry::VelocityBody& velocity_body) {
    // Placeholder for transformation matrix R; should be defined based on your system
    double R[3][3] = { /* populate with appropriate values */ };
    double velocity_ned_arr[3] = { velocity_ned.north_m_s, velocity_ned.east_m_s, velocity_ned.down_m_s };
    
    velocity_body.body_x_m_s = R[0][0] * velocity_ned_arr[0] + R[0][1] * velocity_ned_arr[1] + R[0][2] * velocity_ned_arr[2];
    velocity_body.body_y_m_s = R[1][0] * velocity_ned_arr[0] + R[1][1] * velocity_ned_arr[1] + R[1][2] * velocity_ned_arr[2];
    velocity_body.body_z_m_s = R[2][0] * velocity_ned_arr[0] + R[2][1] * velocity_ned_arr[1] + R[2][2] * velocity_ned_arr[2];
}

// Function to collect telemetry data
void collect_telemetry_data(Telemetry& telemetry) {
    // Retrieve and print telemetry data
    auto odometry = telemetry.odometry();
    auto imu = telemetry.imu();
    
    std::map<std::string, std::string> data;
    data["odometry_position_body_x_m"] = std::to_string(odometry.position_body().x_m);
    data["odometry_position_body_y_m"] = std::to_string(odometry.position_body().y_m);
    data["odometry_position_body_z_m"] = std::to_string(odometry.position_body().z_m);
    data["odometry_velocity_body_x_m_s"] = std::to_string(odometry.velocity_body().x_m_s);
    data["odometry_velocity_body_y_m_s"] = std::to_string(odometry.velocity_body().y_m_s);
    data["odometry_velocity_body_z_m_s"] = std::to_string(odometry.velocity_body().z_m_s);
    
    // Updated imu acceleration fields
    data["imu_acceleration_forward_m_s2"] = std::to_string(imu.acceleration_frd().forward_m_s2);
    data["imu_acceleration_right_m_s2"] = std::to_string(imu.acceleration_frd().right_m_s2);
    data["imu_acceleration_down_m_s2"] = std::to_string(imu.acceleration_frd().down_m_s2);
    
    print_pretty_map(data);
}

int main() {
    Mavsdk mavsdk;  // Ensure this is initialized correctly
    mavsdk.add_any_system();  // Assuming you have a method to add a system
    auto system = mavsdk.systems().at(0);  // Assuming you have at least one system
    
    auto action = Action(system);
    auto offboard = Offboard(system);
    auto telemetry = Telemetry(system);
    
    // Arm the drone
    action.arm().wait();
    
    // Start Offboard mode
    offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0)).wait();
    offboard.start().wait();
    
    // Collect telemetry data
    collect_telemetry_data(telemetry);
    
    // Stop Offboard mode
    offboard.stop().wait();
    
    // Disarm the drone
    action.disarm().wait();
    
    return 0;
}
