#!/usr/bin/env python

import sys
import time
import math
import threading

import messages

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase # Imports for attitude

def pub():
    msg.publish()

def sub():
    while True:
        print('Hi')
        time.sleep(0.5)

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

msg = messages.Messages(master)

# Change message interval
msg.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 5)
msg.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 5)

t1 = threading.Thread(target=pub, name='t1')
# t2 = threading.Thread(target=sub, name='t2')
t1.start()
# t2.start()

