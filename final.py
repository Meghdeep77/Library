from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from geopy.distance import geodesic




connection_string = 'udp:127.0.0.1:14551'
sitl = None



if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()



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


def hover_at_altitude(current_loc,target_altitude):
    temp = LocationGlobalRelative(current_loc.lat, current_loc.lon, target_altitude)
    print("Hovering at {} meters".format(target_altitude))
    vehicle.simple_goto(temp)
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print("Altitude: ", current_altitude)
        print_current_coordinates()
        print("Target Coordinates: Lat={}, Lon={}, Alt={}".format(
            temp.lat, temp.lon, temp.alt
        ))
        if target_altitude - 0.1 <= current_altitude <= target_altitude + 0.1:
            print("Altitude within threshold. Hovering.")
            time.sleep(5)
            break

        time.sleep(1)
def print_current_coordinates():
    current_location = vehicle.location.global_relative_frame
    print("Current Coordinates: Lat={}, Lon={}, Alt={}".format(
        current_location.lat, current_location.lon, current_location.alt
    ))
def get_distance_metres(location1, location2):
    coords1 = (location1.lat, location1.lon)
    coords2 = (location2.lat, location2.lon)
    return geodesic(coords1, coords2).meters

#vehicle.wait_ready('home_location', timeout=10)
launch_location = vehicle.location.global_relative_frame

launch_latitude = launch_location.lat
launch_longitude = launch_location.lon
launch_altitude = launch_location.alt
launch  = LocationGlobalRelative(launch_latitude, launch_longitude , launch_altitude)
print("Launch Coordinates: Lat={}, Lon={}, Alt={}".format(
        launch_latitude, launch_longitude, launch_altitude
    ))
arm_and_takeoff(10)
print_current_coordinates()
print("Set default/target airspeed to 3")
vehicle.airspeed = 10

print("Going towards first point ...")
point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
vehicle.simple_goto(point1)


while True:
    print_current_coordinates()
    current_location = vehicle.location.global_relative_frame
    distance_to_target = get_distance_metres(current_location, point1)
    print("Distance to target: ", distance_to_target)
    if distance_to_target < 1:
        print("Reached the first point.")
        break
    time.sleep(1)


print("Going towards second point (groundspeed set to 20 m/s) ...")
point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
vehicle.simple_goto(point2, groundspeed=20)


while True:
    print_current_coordinates()
    current_location = vehicle.location.global_relative_frame
    distance_to_target = get_distance_metres(current_location, point2)
    print("Distance to target: ", distance_to_target)
    if distance_to_target < 1:
        print("Reached the second point.")
        break
    time.sleep(1)

print("Going towards Third point (groundspeed set to 20 m/s) ...")
point3 = LocationGlobalRelative(-35.364314, 149.17022258, 20)
vehicle.simple_goto(point3, groundspeed=20)

# Wait until the drone reaches the second point
while True:
    print_current_coordinates()
    current_location = vehicle.location.global_relative_frame
    distance_to_target = get_distance_metres(current_location, point3)
    print("Distance to target: ", distance_to_target)
    if distance_to_target < 1:
        print("Reached the third point.")
        hover_at_altitude(current_location,5)
        break
    time.sleep(1)

print("Going towards Fourth point (groundspeed set to 20 m/s) ...")
point4 = LocationGlobalRelative(-35.368394, 149.172178, 20)
vehicle.simple_goto(point4, groundspeed=20)

# Wait until the drone reaches the second point
while True:
    print_current_coordinates()
    current_location = vehicle.location.global_relative_frame
    distance_to_target = get_distance_metres(current_location, point4)
    print("Distance to target: ", distance_to_target)
    if distance_to_target < 1:
        print("Reached the Fourth point.")
        break
    time.sleep(1)
# Return to Launch (RTL)
print("Returning to Launch")
print("Launch Coordinates: Lat={}, Lon={}, Alt={}".format(
        launch_latitude, launch_longitude, launch_altitude
    ))
vehicle.simple_goto(launch, groundspeed=20)
while True:
    print_current_coordinates()
    current_location = vehicle.location.global_relative_frame
    distance_to_target = get_distance_metres(current_location, launch)
    print("Distance to target: ", distance_to_target)
    if distance_to_target < 1:
        print("Reached the launch point.")
        break
    time.sleep(1)
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