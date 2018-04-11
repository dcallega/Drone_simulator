from dronekit import Vehicle, VehicleMode, connect
from MyVehicle import MyVehicle
import time

MODE = "STILL"

batt_log_file = open('logs/batt_fly_still_' + str(time.time()) + '.csv', "w", 0)
batt_log_file.write("#" + ",".join([MODE, str(0), str(0)]) + "\n")
batt_log_file.write(",".join(["Time", "Voltage", "Current", "Level"]) + "\n")

v = connect('/dev/ttyUSB0', baud=57600, wait_ready=True, vehicle_class=MyVehicle)

v.arm_and_takeoff(3)

open_file = batt_log_file
vehicle = v
timeout = 1000
start_time = time.time()
end_time = start_time + timeout
now = time.time()

while now < end_time and vehicle.battery.level > 25:
    try:
        open_file.write(','.join([str(e) for e in [now, vehicle.battery.voltage, vehicle.battery.current, vehicle.battery.level]]) + "\n")
        time.sleep(1)
        now = time.time()
    except Exception as e:
        print(e)
        break
open_file.close()
v.land()
