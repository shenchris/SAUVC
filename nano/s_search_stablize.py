#!/usr/bin/env python

"""
stablize mode only
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
mode = 'STABILIZE'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

gate_found = False

try:
    # stablize
    time.sleep(2)

    ## S-shape search for gate
    # init
    if not gate_found:
        # turn right first
        for i in range(3):
            send_manual_control(0,0,500,400)
            time.sleep(1)
        direction = 1
        count = 1

    while not gate_found:   # cha7nge direction each () second
        if gate_found:
            break
        if count == 50:     # if gate not found in () second, stop it
            break 

        if (count % 6 == 0):
            direction *= -1
        
        send_manual_control(200,0,500,400 * direction) # move forward and change angle
        time.sleep(1)
        print(count,' ', direction)
        count += 1

    # Disarm
    time.sleep(3)   # Wait 3 sec to disarm
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')

except KeyboardInterrupt:
    # Disarm
    time.sleep(3)   # Wait 3 sec to disarm
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')