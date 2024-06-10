import dronekit
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import numpy as np
from datetime import datetime

DRONE_MOVE_FORWARD = 0
DRONE_MOVE_BACKWARD = 1
DRONE_MOVE_RIGHT = 2
DRONE_MOVE_LEFT = 3
DRONE_MOVE_DOWN = 4
DRONE_MOVE_UP = 5

DRONE_DIRECTIONS = [
    DRONE_MOVE_FORWARD,
    DRONE_MOVE_BACKWARD,
    DRONE_MOVE_RIGHT,
    DRONE_MOVE_LEFT,
    DRONE_MOVE_DOWN,
    DRONE_MOVE_UP
]


class Drone:
    __vehicle: dronekit.Vehicle = None
    __takeoff_alt: int = None
    print_status: bool = True

    def __init__(self, connection_string: str, wait_ready=True, print_status=True):
        self.__vehicle = connect(connection_string, wait_ready=wait_ready)
        self.print_status = print_status

    def close(self):
        self.__vehicle.close()

    def setmode(self, mode: str):
        mode = mode.upper()
        self.__vehicle.mode = VehicleMode(mode)

        while not self.__vehicle.mode.name == mode:
            if self.print_status is True:
                print(" Waiting for mode change ...")
            time.sleep(1)

    def arm(self):
        if self.print_status is True:
            print("Basic pre-arm checks")

        while not self.__vehicle.is_armable:
            print("Waiting for vehicle to initialize")
            time.sleep(1)

        print("Arming motors")
        self.__vehicle.armed = True

        while not self.__vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

    def takeoff(self, altitude: int):
        self.__takeoff_alt = altitude

        self.setmode('GUIDED')
        self.__vehicle.arm()

        print("Taking off!")
        self.__vehicle.simple_takeoff(altitude)

        while True:
            print(" Altitude: ", self.__vehicle.location.global_relative_frame.alt)
            if self.__vehicle.location.global_relative_frame.alt >= altitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

    def land(self):
        self.setmode('LAND')

    def set_groundspeed(self, groundspeed: float):
        self.__vehicle.groundspeed = groundspeed

    def set_airspeed(self, airspeed: float):
        self.__vehicle.airspeed = airspeed

    def get_attitude(self):
        return self.__vehicle.attitude

    def get_altitude(self):
        return self.__vehicle.location.global_relative_frame.alt

    def get_heading(self):
        return self.__vehicle.heading

    @staticmethod
    def create_location(location, lat, lon, alt):
        if location in [LocationGlobal, LocationGlobalRelative]:
            target_location = location(lat, lon, alt)
        else:
            raise Exception('Invalid Location type passed')

        return target_location

    @staticmethod
    def get_distance_metres(location1, location2):
        dlat = location2.lat - location1.lat
        dlong = location2.lon - location1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def goto(self, lat, lon, air_speed=None, ground_speed=None):
        current_location = self.__vehicle.location.global_relative_frame
        target_location = self.create_location(type(current_location), lat, lon, current_location.alt)
        target_distance = self.get_distance_metres(current_location, target_location)
        self.__vehicle.simple_goto(target_location, airspeed=air_speed, groundspeed=ground_speed)

        while self.__vehicle.mode.name == "GUIDED":
            location = self.__vehicle.location.global_relative_frame
            remaining_distance = self.get_distance_metres(location, target_location)
            print("Distance to target: ", remaining_distance)
            if remaining_distance <= target_distance * 0.05:
                print("Reached target")
                time.sleep(5)
                break
            time.sleep(1)

    def get_location(self):
        current_location = self.__vehicle.location.global_relative_frame
        loc = {'latitude': current_location.lat,
               'longitude': current_location.lon,
               'timestamp': str(datetime.now())}
        return loc

    def __send_global_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        duration = int(duration)

        msg = self.__vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x,  # X velocity in NED frame in m/s
            velocity_y,  # Y velocity in NED frame in m/s
            velocity_z,  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        self.__vehicle.send_mavlink(msg)
        time.sleep(1)

    @staticmethod
    def decompose_velocity(vel, heading):
        heading = np.deg2rad(heading)
        vel_x = vel * np.cos(heading)
        vel_y = vel * np.sin(heading)
        return vel_x, vel_y, 0

    def move(self, direction, velocity, duration=1):
        if direction not in DRONE_DIRECTIONS:
            print('Invalid Direction passed')
            return

        vel_x, vel_y, vel_z = 0, 0, 0

        if direction != DRONE_MOVE_DOWN and direction != DRONE_MOVE_UP:
            heading = self.get_heading()
            vel_x, vel_y, vel_z = self.decompose_velocity(velocity, heading)

            if direction == DRONE_MOVE_FORWARD:
                vel_x, vel_y = vel_x, vel_y
            elif direction == DRONE_MOVE_BACKWARD:
                vel_x, vel_y = -vel_x, -vel_y
            elif direction == DRONE_MOVE_RIGHT:
                vel_x, vel_y = -vel_y, vel_x
            elif direction == DRONE_MOVE_LEFT:
                vel_x, vel_y = vel_y, -vel_x

        elif direction == DRONE_MOVE_DOWN:
            vel_z = velocity
        elif direction == DRONE_MOVE_UP:
            vel_z = -velocity

        self.__send_global_velocity(vel_x, vel_y, vel_z, duration=duration)
        time.sleep(1)

    def __condition_yaw(self, target_heading: int, relative=False):
        current_heading = self.get_heading()
        if relative is False:
            cw_angle = (target_heading - current_heading + 360) % 360
            ccw_angle = (current_heading - target_heading + 360) % 360

            if cw_angle < ccw_angle:
                angle = cw_angle
                yaw_dir = 1
            else:
                angle = ccw_angle
                yaw_dir = -1
        else:
            angle = abs(target_heading)
            if target_heading < 0:
                yaw_dir = -1
            else:
                yaw_dir = 1

        sleep_time = math.ceil(angle * 10 / 360)

        # create the CONDITION_YAW command using command_long_encode()
        msg = self.__vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            angle,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            yaw_dir,  # param 3, direction -1 ccw, 1 cw
            1,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.__vehicle.send_mavlink(msg)

        time.sleep(sleep_time + 2)  # Add buffer of 2 sec

    def change_heading(self, heading, relative=False):
        self.__condition_yaw(heading, relative=relative)

    def change_altitude_to(self, target_alt: float):
        current_location = self.__vehicle.location.global_relative_frame
        target_location = self.create_location(
            type(current_location),
            current_location.lat,
            current_location.lon,
            target_alt
        )

        self.__vehicle.simple_goto(target_location)

        while self.__vehicle.mode.name == "GUIDED":
            current_location = self.__vehicle.location.global_relative_frame
            remaining_distance = abs(current_location.alt - target_alt)
            print("Distance to target: ", remaining_distance)
            if remaining_distance < 2:
                print("Reached target")
                time.sleep(5)
                break
            time.sleep(1)

    def change_altitude_by(self, difference_altitude: float):
        current_altitude = self.__vehicle.location.global_relative_frame.alt
        target_alt = current_altitude + difference_altitude
        self.change_altitude_to(target_alt)
