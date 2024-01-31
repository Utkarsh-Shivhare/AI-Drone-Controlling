from pymavlink import mavutil

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

# Define desired altitude and waypoint parameters
desired_altitude = 10.0

# Convert desired altitude to centimeters
desired_altitude_cm = int(desired_altitude * 100)
waypoint_seq = 1  # Sequence number for waypoints

target_altitude = 1000  # 10 meters

# Create a MAVLink message to set the desired position
msg = master.mav.set_position_target_global_int_encode(
    0,  # System ID
    0,  # Component ID
    time.time(),  # Timestamp
    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,  # Coordinate frame (Global)
    0b0000111111111000,  # Use only the Z component
    0, 0, 0,  # Latitude, longitude, and altitude (not used in this case)
    0, 0, 0,  # Velocity components (not used in this case)
    0, 0, target_altitude,  # Acceleration components (set the desired altitude here)
    0, 0, 0,  # Attitude components (not used in this case)
    0, 0, 0  # Yaw rate (not used in this case)
)

# Send the message
master.mav.send(msg)

# Waypoint commands for takeoff and loiter at 10m
takeoff_command = master.mav.mission_item_encode(
    0,  # Target System
    0,  # Target Component
    waypoint_seq,  # Sequence
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Command (takeoff)
    0, 0, 0, 0, 0, 0, 0, 0, 0, desired_altitude_cm  # Parameters (convert to centimeters)
)

loiter_command = master.mav.mission_item_encode(
    0,  # Target System
    0,  # Target Component
    waypoint_seq + 1,  # Sequence
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame
    mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,  # Command (loiter)
    0, 0, 0, 0, 0, 0, 0, 0, 0, desired_altitude_cm  # Parameters (convert to centimeters)
)

# Send arming command
master.mav.send(arming_command)

# Wait for a few seconds to allow arming to complete (time may vary)
import time
time.sleep(5)

# Send waypoint commands
master.mav.send(takeoff_command)
master.mav.send(loiter_command)

# Monitor drone state and perform additional actions as needed
while True:
    msg = master.recv_msg()
    if msg is not None:
        # Monitor drone state, handle telemetry, and implement safety checks
        pass
