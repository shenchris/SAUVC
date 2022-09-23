#!/usr/bin/env python3

# Description: program for qualification
# Using SetDepth, CV, and P control z_axis when moving and sink
# Don't include CV

import sys
from pymavlink import mavutil
import time
import threading
import math
import rospy
from gate_detection.msg import instruction_gate
import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

# variable setting
Camera_topic = "gate_detection/instruction"

### hard code variable
target_pressure = 1060
Forward_speed_without_CV = 1000
Forward_speed_with_CV = 1000
Sink_second = 1  # time let auv sink at begining


def listener_to_camara():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(Camera_topic, instruction_gate, motion)
    rospy.spin()


def motion(data):
    try:
        while not data.pass_yet:
            if not data.c_gate:
                send_manual_control(Forward_speed_without_CV, 0, 0, 0)
            else:
                send_manual_control(Forward_speed_with_CV, int(data.force / 1.3), 0, 0)

    except KeyboardInterrupt:
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print('Disarmed!')


def send_manual_control(x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        x,  # -1000 to 1000, static=0, backward<0, forward>0
        y,  # -1000 to 1000, static=0, left<0, right>0
        z,  # 0 to 1000, static=500, downward<500, upward>500
        r,  # -1000 to 1000, static=0, anti-clockwise<0, clockwise>0
        0  # useless (for other purpose)
    )


def thread_function():
    global z
    while True:
        try:
            z = master.recv_match(type='SCALED_PRESSURE2', blocking=True).to_dict()["press_abs"]
        except KeyboardInterrupt:
            master.arducopter_disarm()
            master.motors_disarmed_wait()
            print('Disarmed!')


master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
master.wait_heartbeat()

print("Arm Status: " + str(bool(master.motors_armed())))

master.arducopter_arm()
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

mode = 'ALT_HOLD'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)

global z
z = 0
print("Starting Thread")
x = threading.Thread(target=thread_function)
x.start()
time.sleep(0.5)

try:
    print("Sinking into water")
    # set depth

    for i in range(Sink_second * 20):
        send_manual_control(0, 0, 500 + int((math.floor(z) - target_pressure) * 3), 0)
        time.sleep(0.05)
    listener_to_camara()

except KeyboardInterrupt:
    # Disarm
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')
