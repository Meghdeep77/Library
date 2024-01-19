from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Set up option parsing to get connection string


connection_string = 'udp:127.0.0.1:14551'
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")

    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True


    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)

        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def print_current_coordinates():
    current_location = vehicle.location.global_relative_frame
    print("Current Coordinates: Lat={}, Lon={}, Alt={}".format(
        current_location.lat, current_location.lon, current_location.alt
    ))

#vehicle.wait_ready('home_location', timeout=10)
current_location = vehicle.location.global_relative_frame

launch_latitude = current_location.lat
launch_longitude = current_location.lon
launch_altitude = current_location.alt
launch  = LocationGlobalRelative(launch_latitude, launch_longitude , launch_altitude)
arm_and_takeoff(10)
print_current_coordinates()
print("Set default/target airspeed to 3")
vehicle.airspeed = 10

print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
vehicle.simple_goto(point1)



time.sleep(30)
print_current_coordinates()
print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
vehicle.simple_goto(point2, groundspeed=20)


time.sleep(30)
print_current_coordinates()
# Return to Launch (RTL)
print("Returning to Launch")
print("Launch Coordinates: Lat={}, Lon={}, Alt={}".format(
        launch_latitude, launch_longitude, launch_altitude
    ))
vehicle.simple_goto(launch, groundspeed=20)
time.sleep(30)
while vehicle.location.global_relative_frame.alt > 0:
    print_current_coordinates()
    time.sleep(1)


while not vehicle.is_armable:
    time.sleep(1)

print("Landing...")
vehicle.mode = VehicleMode("LAND")


while vehicle.location.global_relative_frame.alt > 0:
    print_current_coordinates()
    time.sleep(1)

print("Vehicle has landed.")

# Close and Cleanup
print("Close vehicle object")
vehicle.close()

if sitl:
    sitl.stop()