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

# arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()
print("arm")

# set the desired operating mode
DEPTH_HOLD = 'ALT_HOLD'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)
print("set mode")

t = time.time()
while True:
    try:
        msg = master.recv_match(type='SCALED_PRESSURE2', blocking=True).to_dict()["press_abs"]
        print(msg)
        t = time.time()
        print(time.time() - t)

    except KeyboardInterrupt:
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print('Disarmed!')
