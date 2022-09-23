import threading
import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print("connect")


def thread_function():
    global yaw, pressure, master
    while True:
        try:
            msg = master.recv_match(type=['SCALED_PRESSURE', 'ATTITUDE'], blocking=True).to_dict()
            if msg['mavpackettype'] == 'SCALED_PRESSURE':
                pressure = msg['press_abs']
            elif msg['mavpackettype'] == 'ATTITUDE':
                yaw = msg['yaw']


        except KeyboardInterrupt:
            master.arducopter_disarm()
            master.motors_disarmed_wait()

global yaw, pressure
yaw = None
pressure = None
x = threading.Thread(target=thread_function)
x.start()
time.sleep(0.5)

for i in range(1000):
    try:
        time.sleep(0.1)
    # arm ArduSub autopilot and wait until confirmed
    except KeyboardInterrupt:
        # Disarm
        time.sleep(3)
        master.arducopter_disarm()
        print("Waiting for the vehicle to disarm")
        # Wait for disarm
        master.motors_disarmed_wait()
        print('Disarmed!')
