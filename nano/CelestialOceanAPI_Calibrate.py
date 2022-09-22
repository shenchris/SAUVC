import time

from pymavlink import mavutil

from CelestialOceanAPI.CelestialOceanAPI import CelestialOceanAPI

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# Wait a heartbeat before sending commands
master.wait_heartbeat()

coAPI = CelestialOceanAPI(master)

#coAPI.manipulator.grab()
coAPI.cameraPedestal.setAngle(0)
print("Camera Pedestal Reset to 0")

while True:

    # Up
    time.sleep(1)
    for angle in range(-90, 90, 1):
        try:
            coAPI.cameraPedestal.setAngle(angle)
            time.sleep(0.05)
        except Exception as e:
            print(e)

    # Down
    time.sleep(1)
    for angle in range(90, -90, -1):
        try:
            coAPI.cameraPedestal.setAngle(angle)
            time.sleep(0.05)
        except Exception as e:
            print(e)

    time.sleep(1)
    coAPI.manipulator.grab()

    time.sleep(1)
    coAPI.manipulator.release()

