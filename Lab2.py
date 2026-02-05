from pymavlink import mavutil
import time
import sys


# Connect to SITL
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
connection.wait_heartbeat()

print("Connected to SITL. :D")

def read_sensors(connection):
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        latitude = msg.lat / 1e7          # Convert to degrees
        longitude = msg.lon / 1e7         # Convert to degrees
        altitude = msg.relative_alt / 1000.0  # Convert to meters

        print(f"Position: Lat={latitude}, Lon={longitude}, Alt={altitude} m, tipo: {type(msg)}")
    else:
        print("Failed to read sensor data.")

# Call the function
read_sensors(connection)


def arm_vehicle(connection):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # Arm
        0, 0, 0, 0, 0, 0
    )
    print("Drone armed.")

# Arm the drone (moved to mission sequence)
# arm_vehicle(connection)

def takeoff(connection, altitude):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # Confirmation
        0, 0, 0, 0, 0, 0, altitude  # Desired altitude
    )
    print(f"Taking off to {altitude} meters.")


# Take off to 10 meters (moved to mission sequence)
# takeoff(connection, 10)

def set_mode(connection, mode):
    connection.set_mode(mode)
    print(f"Mode changed to {mode}.")

def go_to_position(connection, lat, lon, alt):
    connection.mav.set_position_target_global_int_send(
        0,  # Timestamp
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Relative altitude frame
        0b110111111000,  # Ignore velocity and acceleration
        int(lat * 1e7),  # Latitude as integer
        int(lon * 1e7),  # Longitude as integer
        alt,  # Desired altitude in meters
        0, 0, 0,  # Velocities
        0, 0, 0,  # Accelerations
        0, 0  # Yaw and Yaw rate
    )
    print(f"Moving drone to Lat={lat}, Lon={lon}, Alt={alt} m")

# Mission logic moved to end of file to ensure all functions are defined


def read_battery_status(connection):
    msg = connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=5)
    if msg:
        voltage = msg.voltages[0] / 1000.0  # Convert mV to V
        current = msg.current_battery / 100.0  # Convert cA to A
        remaining = msg.battery_remaining  # Remaining battery percentage
        print(f"Battery: {voltage:.2f} V, Current: {current:.2f} A, Remaining: {remaining}%")
        return remaining
    else:
        print("Failed to get battery status.")
        return None

# Read battery status
read_battery_status(connection)

def land_vehicle(connection):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("Landing command sent.")


# Validar modo GUIDED
set_mode(connection, 'GUIDED')

# Armar y Despegar antes de la misión
arm_vehicle(connection)
takeoff(connection, 10)
time.sleep(10) # Esperar a que despegue

# Waypoint 1
print("Yendo al Waypoint 1")
go_to_position(connection, -35.362956, 149.164857, 10)
time.sleep(20)
read_sensors(connection)
if (bat := read_battery_status(connection)) is not None and bat <= 20:
    land_vehicle(connection)
    sys.exit()

# Waypoint 2
print("Yendo al Waypoint 2")
go_to_position(connection, -35.362212, 149.165071, 10)
time.sleep(20)
read_sensors(connection)
if (bat := read_battery_status(connection)) is not None and bat <= 20:
    land_vehicle(connection)
    sys.exit()

# Waypoint 3
print("Yendo al Waypoint 3")
go_to_position(connection, -35.362791, 149.165433, 10)
time.sleep(20)
read_sensors(connection)
if (bat := read_battery_status(connection)) is not None and bat <= 20:
    land_vehicle(connection)
    sys.exit()

# Waypoint 4
print("Yendo al Waypoint 4")
go_to_position(connection, -35.363250, 149.165245, 10)
time.sleep(20)
read_sensors(connection)
if (bat := read_battery_status(connection)) is not None and bat <= 20:
    land_vehicle(connection)
    sys.exit()

# Aterrizar
land_vehicle(connection)

# Monitor battery and land if necessary
# while True:
#     battery_remaining = read_battery_status(connection)
#     if battery_remaining is not None and battery_remaining <= 20:
#         print("Low battery! Initiating emergency landing.")
#         land_vehicle(connection)
#         break

#     time.sleep(5)  # Check every 5 seconds
