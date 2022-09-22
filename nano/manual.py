#!/usr/bin/env python

"""
manual control
"""

import sys
from pymavlink import mavutil
import time

def send_manual_control(x,y,z,r):
    master.mav.manual_control_send(
        master.target_system,
        x,	  # -1000 to 1000, static=0, backward<0, forward>0
        y,    # -1000 to 1000, static=0, left<0, right>0
        z,    # 0 to 1000, static=500, downward<500, upward>500
        r,    # -1000 to 1000, static=0, anti-clockwise<0, clockwise>0
        0    # useless (for other purpose)
    )


### Start program ###

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)    
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Arm
master.arducopter_arm()
print("Waiting for the vehicle to arm")
# Wait to arm
master.motors_armed_wait()
print('Armed!')

# Choose a mode
mode = 'MANUAL'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

try:

    for i in range(1):
        send_manual_control(100,0,600,0)
        time.sleep(1)

    # Disarm
    time.sleep(2)
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')

except KeyboardInterrupt:
    # Disarm
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')
