from dronekit import Vehicle, VehicleMode, connect
from MyVehicle import MyVehicle
import time
import thread

speed = 500
duration = 2.4

batt_log_file = open('logs/batt_fly_moving_' + str(time.time()) + '.csv', "w", 0)

v = connect('/dev/ttyUSB0', baud=57600, wait_ready=True, vehicle_class=MyVehicle)

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
        open_file.write(','.join([str(e) for e in [now, vehicle.battery.voltage, vehicle.battery.current, vehicle.battery.level]]) + "\n")
        if iteration % 2 == 0:
            vehicle.move(1, 0, 0, duration+0.5, speed)
        if iteration % 2 == 1:
            vehicle.move(-1, 0, 0, duration, speed)
        vehicle.move(0, 0, 0, 0.5)
        now = time.time()
        iteration += 1
    except Exception as e:
        print(e)
        break
open_file.close()
v.land()
