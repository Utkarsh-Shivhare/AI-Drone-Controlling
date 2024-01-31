from pymavlink import mavutil
import time

# Define the serial port and baud rate
serial_port = '/dev/serial0'  # Update this with your actual serial port
baud_rate = 921600  # Update this with your desired baud rate

def is_armable(connection):
    # Check if the vehicle is armable by reading the pre-arm checks result
    if 'ARMING_CHECK' in connection.messages:
        prearm_check_result = connection.messages['ARMING_CHECK']
        return prearm_check_result.prearm_check == 0  # 0 means it's armable

    return False
# Initialize the MAVLink connection with the specified baud rate
master = mavutil.mavlink_connection(serial_port, baud=baud_rate)

# Wait for the connection to be established
master.wait_heartbeat()

# Send a takeoff command
command = master.mav.command_long_encode(
    1,                  # System ID
    1,                  # Component ID
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Command
    0,                  # Confirmation
    0, 0, 0, 0, 0, 0, 0, 0, 0, 10  # Parameters: pitch, yaw, latitude, longitude, altitude (meters)
)

master.mav.send(command)

# Close the connection when done
master.close()


current_altitude = vehicle.location.global_relative_frame.alt
print(f"Reached target altitude of {current_altitude} meters")
if abs(current_altitude - target_altitude) < 0.5:
    print(f"Reached target altitude of {target_altitude} meters")
    break  
