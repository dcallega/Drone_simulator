from dronekit import Vehicle, VehicleMode, connect
from MyVehicle import MyVehicle
import time
import thread

speed = 500
duration = 2.4
distance = 10

MODE = "MOVE"

batt_log_file = open('logs/batt_fly_moving_' + str(time.time()) + '.csv', "w", 0)
batt_log_file.write("#" + ",".join([MODE, str(speed), str(distance)]) + "\n")
batt_log_file.write(",".join(["Time", "Voltage", "Current", "Level"]) + "\n")

v = connect('/dev/ttyUSB0', baud=57600, wait_ready=True, vehicle_class=MyVehicle)
ping, pong = v.location.global_relative_frame, v.get_location_meters_uav(10,0)

v.arm_and_takeoff(3)

open_file = batt_log_file
vehicle = v
timeout = 10000
start_time = time.time()
end_time = start_time + timeout
now = time.time()
iteration = 0
while now < end_time and vehicle.battery.level > 25:
    try:
        if iteration % 2 == 0:
            target_location = pong
        else:
            target_location = ping

        v.simple_goto(target_location)
        target_distance = v.get_distance_meters(v.location.global_relative_frame, target_location)
        rem_dist_list = []
        while v.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
            open_file.write(','.join([str(e) for e in [now, vehicle.battery.voltage, vehicle.battery.current,
                                                       vehicle.battery.level]]) + "\n")
            remainingDistance = v.get_distance_meters(v.location.global_frame, target_location)
            rem_dist_list.append(remainingDistance)
            print "Distance to target: ", remainingDistance
            if len(rem_dist_list) > 3:
                one, two, three = rem_dist_list[-3], rem_dist_list[-2], rem_dist_list[-1]
                if (two + three) * 0.5 * 0.95 < one < (two + three) * 0.5 * 1.05:
                    v.simple_goto(target_location)
                    rem_dist_list = []
                    print("Resend command")
            if remainingDistance <= target_distance * 0.05:  # Just below target, in case of undershoot.
                print "Reached target"
                rem_dist_list = []
                break;
            time.sleep(0.5)

        now = time.time()
        iteration += 1
    except Exception as e:
        print(e)
        break
open_file.close()
v.land()
