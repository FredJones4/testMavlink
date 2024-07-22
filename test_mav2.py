from pymavlink import mavutil

# Connect to the Pixhawk
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for the heartbeat message to find the system ID
master.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" % (master.target_system, master.target_component))

# Function to request control of the MAV
def request_control(target_system, control_request, version, passkey):
    master.mav.change_operator_control_send(
        target_system,
        control_request,
        version,
        passkey.encode('utf-8')
    )
    # Wait for ACK
    while True:
        msg = master.recv_match(type='CHANGE_OPERATOR_CONTROL_ACK', blocking=True)
        if msg:
            if msg.ack == 0:
                print("Control granted")
            elif msg.ack == 1:
                print("Control denied: Wrong passkey")
            elif msg.ack == 2:
                print("Control denied: Unsupported passkey encryption method")
            elif msg.ack == 3:
                print("Control denied: Already under control")
            break

# Request control of the MAV (use appropriate target_system, control_request, version, and passkey)
request_control(target_system=1, control_request=0, version=0, passkey='')

# Function to send a hover command
def send_hover_command():
    # Example hover command, specific implementation may vary
    # The command below is a placeholder and should be replaced with actual hover command
    master.mav.command_long_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,  # command
        0,  # confirmation
        0, 0, 0, 0, 0, 0, 0  # parameters
    )
    print("Hover command sent")

# Send a hover command
send_hover_command()
