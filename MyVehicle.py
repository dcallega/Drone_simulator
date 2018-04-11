from dronekit import Vehicle, VehicleMode, connect, LocationGlobal, LocationGlobalRelative
from threading import Lock
import time
import math
from pymavlink import mavutil


# Class constants
OVERRIDE_NEUTRAL = 1500
AND_GUIDED = True
LEFT = 1
RIGHT = 2
UP = 3
DOWN = 4
FORWARD = 5
BACKWARDS = 6


class MyVehicle(Vehicle):
    def __init__(self, *args):
        super(MyVehicle, self).__init__(*args)

        self.verbose = False
        self.lock = Lock()
        # Create an Vehicle.raw_imu object with initial values set to None.
        
        self.overridevalue = [OVERRIDE_NEUTRAL]*3
        self.data = [None, None, None]
        self.duration = 1
        self.speed = 200

    def rc_override(self, data):
        if self.mode == VehicleMode("LAND"):
            return
        self.mode = VehicleMode('LOITER')
        now = time.time()
        # while time.time() - now < duration:
        self.channels.overrides = {1: data[0], 2: data[1], 3: data[2]}
        if self.verbose:
            print("Mode = " + str(self.mode) + "\nRChannels = {1: %d, 2: %d, 3: %d}"%(data[0], data[1], data[2]))

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        if self.verbose:
            print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        print(self.mode)
        while not self.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(3)
            self.mode = VehicleMode("GUIDED")
        if self.verbose:
            print("Arming motors")
        # Copter should arm in GUIDED mode
        self.mode = VehicleMode("GUIDED")
        self.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.armed:
            print(" Waiting for arming...", self.mode)
            time.sleep(3)
            if not self.armed:
                self.armed = True

        print("Taking off!")
        self.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            print(aTargetAltitude * 0.95)
            if self.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def takeoff(self, altitude= 3, takeoff=True):
        print("Taking off of " + str(altitude) + " meters")
        self.arm_and_takeoff(altitude)
    
    def set_mode(self, m = VehicleMode("LAND")):
        if type(m) == type("m"):
            self.mode = VehicleMode(m)
        elif type(m) == type(VehicleMode("GUIDED")):
            self.mode = m
        else:
            raise TypeError("Trying to set VehicleMode with " + str(type(m)))
        self.channels.override = {1: self.data[0], 2: self.data[1], 3: self.data[2]}
    
    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration=1):
        """
        Move vehicle in direction based on specified velocity vectors and
        for the specified duration.

        This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
        velocity components 
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
        
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version 
        (sending the message multiple times does not cause problems).
        
        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        
        msg = self.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        
        d = int(10*duration)
        # send command to vehicle on 10 Hz cycle
        for x in range(0,d):
            self.send_mavlink(msg)
            time.sleep(0.1)
            
    def move_rel_ned(self, x_d,y_d,z_d, duration=1):
        print("-----------------------move_rel_ned(" + ",".join([str(e) for e in [x_d, y_d, z_d, duration]])
              + ")-----------------------")
        theta = math.radians(self.heading)
        c, s = math.cos(theta), math.sin(theta)
        self.send_ned_velocity(-y_d*s+x_d*c, y_d*c+x_d*s, z_d, duration)
        self.send_ned_velocity(0, 0, 0, 0.1)
        
    def move(self, lr, fb, ud, duration=1, speed=100):
        if speed > 5 or speed < -5:
            speed_used = int(speed/100.)
        else:
            speed_used = speed
        norm = math.sqrt(lr**2+fb**2+ud**2)*1.0
        if norm == 0:
            self.move_rel_ned(fb, lr, ud, duration)
        else:
            x_n = (fb/norm)*speed_used
            y_n = (lr/norm)*speed_used
            z_n = -(ud/norm)*speed_used
            self.move_rel_ned(x_n, y_n, z_n, duration)

    def go_to_position_target_local_uav(self, x_d, y_d, z_d):
        theta = math.radians(self.heading)
        c, s = math.cos(theta), math.sin(theta)
        self.goto_position_target_local_ned(-y_d * s + x_d * c, y_d * c + x_d * s, z_d)
    
    def land(self, land=True):
        self.emergency_land()

    def flex_move(self, *args):
        # Constants
        lr, fb, ud = 0, 0, 0
        for e in args:
            if e in [LEFT, RIGHT] and lr == 0:
                lr = e
            elif e in [UP, DOWN] and ud == 0:
                ud = e
            elif e in [FORWARD, BACKWARDS] and fb == 0:
                fb = e
            else:
                raise ValueError("Error parsing args in flex_move " + ",".join(args))
        lr, fb, ud = from62bin(lr, fb, ud)
        self.move(lr, fb, ud)

    def emergency_land(self):
        print(str(time.time()) + " - LANDING IN RTL")
        self.mode = VehicleMode('RTL')

    def goto_position_target_local_ned(self, north, east, up):
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.
        It is important to remember that in this frame, positive altitudes are entered as negative
        "Down" values. So if down is "10", this will be 10 metres below the home altitude.
        Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
        ignored. For more information see:
        http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned
        See the above link for information on the type_mask (0=enable, 1=ignore).
        At time of writing, acceleration and yaw bits are ignored.
        """
        down = -1*up
        msg = self.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            north, east, down,  # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.send_mavlink(msg)

    def set_speed(self, target_speed=0):
        self.groundspeed = target_speed

    def get_location_metres(original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.
        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
        else:
            raise Exception("Invalid Location object passed")

        return targetlocation;

    def get_location_meters(self, original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.
        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
        else:
            raise Exception("Invalid Location object passed")

        return targetlocation;

    def get_distance_meters(self, aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def get_location_meters_uav(self, forward, right):
        theta = math.radians(self.heading)
        c, s = math.cos(theta), math.sin(theta)
        north, east = -1*forward*s + right * c, forward * c + right * s
        return self.get_location_meters(self.location.global_relative_frame, north, east)

    def goto_position_target_global_int(self, aLocation):
        """
        Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified location.
        """
        msg = self.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111111000,  # type_mask (only speeds enabled)
            aLocation.lat * 1e7,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            aLocation.lon * 1e7,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            aLocation.alt,
            # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0,  # X velocity in NED frame in m/s
            0,  # Y velocity in NED frame in m/s
            0,  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle
        self.send_mavlink(msg)


def from62bin(lr, fb, ud):
    if lr == LEFT:
        lr = -1
    elif lr == RIGHT:
        lr = 1
    if fb == BACKWARDS:
        fb = -1
    elif fb == FORWARD:
        fb = 1
    if ud == DOWN:
        ud = -1
    elif ud == UP:
        ud = 1
    return lr, fb, ud
    
    
def main():
    real = False
    if real:
        v = connect('/dev/ttyUSB0', baud=57600, wait_ready=True, vehicle_class=MyVehicle)
    else:
        v = connect('127.0.0.1:14551', wait_ready=True, vehicle_class=MyVehicle)
    #v = connect('127.0.0.1:14551', wait_ready = True, vehicle_class=MyVehicle)
    v.arm_and_takeoff(3)
    print(v.heading)
    ping, pong = v.location.global_relative_frame, v.get_location_meters_uav(10,0)

    for i in range(10):
        try:
            if i%2==0:
                target_location = pong
            else:
                target_location = ping
            v.simple_goto(target_location)
            target_distance = v.get_distance_meters(v.location.global_relative_frame, target_location)
            rem_dist_list = []
            while v.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
                remainingDistance = v.get_distance_meters(v.location.global_frame, target_location)
                rem_dist_list.append(remainingDistance)
                print "Distance to target: ", remainingDistance
                if len(rem_dist_list) > 3:
                    one, two, three = rem_dist_list[-3], rem_dist_list[-2], rem_dist_list[-1]
                    if (two+three)*0.5*0.95 < one < (two+three)*0.5*1.05:
                        v.simple_goto(target_location)
                        rem_dist_list = []
                        print("Resend command")
                if remainingDistance <= target_distance * 0.05:  # Just below target, in case of undershoot.
                    print "Reached target"
                    rem_dist_list = []
                    break;
                time.sleep(1)
        except Exception as e:
            print(e)
            break
    v.land()

if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

