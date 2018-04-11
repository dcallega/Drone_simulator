from dronekit import Vehicle, VehicleMode, connect
from MyVehicle import MyVehicle
import time
import thread
import cv2

def exp_avg(list, alpha=0.5):
    res = 0
    for e in list:
        if e is 0:
            res = e
        elif e is None:
            return res
        res = alpha*e + (1-alpha)*res


def image_analysis(occ_fract=1):
    img = cv2.imread("img/faces.jpg")
    face_cascade = cv2.CascadeClassifier('haar_frontal_face.xml')
    time2det = [None]*3

    for i in range(1000000):
        start = time.time()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.01, 6)
        if None in time2det:
            for idx, e in enumerate(time2det):
                if e is None:
                    time2det[idx] = time.time() - start
                    break
                else:
                    continue
        if i % 100 > int(occ_fract*100):
            time.sleep(exp_avg(time2det))


MODE = "STILL_COMPUTING"

batt_log_file = open('logs/batt_fly_still_' + str(time.time()) + '.csv', "w", 0)
batt_log_file.write("#" + ",".join([MODE, str(0), str(0)]) + "\n")
batt_log_file.write(",".join(["Time", "Voltage", "Current", "Level"]) + "\n")

v = connect('/dev/ttyUSB0', baud=57600, wait_ready=True, vehicle_class=MyVehicle)

v.arm_and_takeoff(3)

open_file = batt_log_file
vehicle = v
timeout = 3600
start_time = time.time()
end_time = start_time + timeout
now = time.time()
thread.start_new_thread(image_analysis(1))
while now < end_time and vehicle.battery.level > 25:
    try:
        open_file.write(','.join([str(e) for e in [now, vehicle.battery.voltage, vehicle.battery.current, vehicle.battery.level]]) + "\n")
        time.sleep(1)
        now = time.time()
    except Exception as e:
        break
open_file.close()
v.land()
