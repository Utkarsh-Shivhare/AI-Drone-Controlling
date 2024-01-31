
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
from math import radians, sin, cos, sqrt, atan2
def calculate_distance(location1, location2):
    # Radius of the Earth in meters
    earth_radius = 6371000.0

    # Convert latitude and longitude from degrees to radians
    lat1 = radians(location1.lat)
    lon1 = radians(location1.lon)
    lat2 = radians(location2.lat)
    lon2 = radians(location2.lon)

    # Differences in latitude and longitude
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Haversine formula to calculate distance
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = earth_radius * c  # Distance in meters

    return distance

# Set up option parsing to get connection string

# x=VehicleMode("RTL_LAND")
# connection_string="udp:127.0.0.1:14551"
connection_string = '/dev/serial0'  # Update this with your actual serial port
baud_rate = 921600  # Update this with your desired baud rate
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string,baud=baud_rate, wait_ready=True)
home_location = vehicle.location.global_frame
target_location = home_location

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.6
    SMOOTH_TAKEOFF_THRUST = 0.6

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    vehicle.channels.overrides['3'] = 1100
    time.sleep(1)
    vehicle.channels.overrides['3']=1000

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        target_location.alt = current_altitude  # Use the current altitude
        vehicle.simple_goto(target_location)
        while calculate_distance(vehicle.location.global_frame,target_location) > 0.1:
            time.sleep(0.2)
        set_attitude(thrust = thrust)
        time.sleep(0.2)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    
    attitude = vehicle.attitude

        # Extract roll, pitch, and yaw angles
    roll_angle = attitude.roll
    pitch_angle = attitude.pitch
    yaw_angle = attitude.yaw

    # Print the angles
    print(f"Roll: {roll_angle}, Pitch: {pitch_angle}, Yaw: {yaw_angle}")

    start = time.time()
    while time.time() - start < duration:
        current_altitude = vehicle.location.global_relative_frame.alt
        target_location.alt = current_altitude  # Use the current altitude
        vehicle.simple_goto(target_location)
        while calculate_distance(vehicle.location.global_frame,target_location) > 0.1:
            time.sleep(0.2)
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

# Take off 2.5m in GUIDED_NOGPS mode.
arm_and_takeoff_nogps(2.5)

# Hold the position for 3 seconds.
print("Hold position for 3 seconds")
set_attitude(duration = 10)

# Uncomment the lines below for testing roll angle and yaw rate.
# Make sure that there is enough space for testing this.

# set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
# set_attitude(yaw_rate = 30, thrust = 0.5, duration = 3)

# Move the drone forward and backward.
# Note that it will be in front of original position due to inertia.
print("Move forward")
# set_attitude(pitch_angle = -5, thrust = 0.5, duration = 3.21)

print("Move backward")
# set_attitude(pitch_angle = 5, thrust = 0.5, duration = 3)


print("Setting LAND mode...")
vehicle.mode=VehicleMode("RTL")
time.sleep(10)
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")