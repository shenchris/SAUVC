#!/usr/bin/env python

"""
control to target yaw
"""

import time
import sys
import math
from pymavlink import mavutil

class YAW_PID:
    def __init__(self):
        # Todo: testing & change to suitable value
        self.Kp = 500
        self.Ki = 30
        self.Kd = 30
        self.max_output = 500
        self.ITerm_max = 10
        self.delta_time = 0.01    # 100Hz
        self.sensitivity = 0.02

        # init
        self.target = 0.0
        self.last_error = 0.0
        self.last_int = 0.0

    def clear(self):
        self.target = 0.0
        self.last_error = 0.0
        self.last_int = 0.0

    # Calculate P, I ,D , and output thruster command
    def update(self, sensor_value):
        delta_time = self.delta_time

        _error = self.target - sensor_value

        # pre-processing
        if abs(_error) < self.sensitivity:
            return 0
        
        if (_error < -math.pi):
            _error += 2 * math.pi
        elif (_error > math.pi):
            _error -= 2 * math.pi

        # calculate P term
        _p = self.Kp * _error

        # calculate I term
        _i = self.Ki * (_error * delta_time + self.last_int)

        # fix max I term
        if (_i < -self.ITerm_max):
            _i = -self.ITerm_max
        elif (_i > self.ITerm_max):
            _i = self.ITerm_max

        self.last_int = _i

        # calculate D term
        _d = self.Kd * (_error - self.last_error) / delta_time
        
        self.last_error = _error

        # calculate output
        output = _p + _i + _d

        if output > self.max_output:
            output = self.max_output
        elif output < -self.max_output:
            output = -self.max_output

        return int(output)

    def set_target(self, goal):
        self.target = goal


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

# Create yaw comtroller
r_controller = YAW_PID()

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavlink.MAVLINK_MSG_ID_ATTITUDE,
    1e6 / 100,
    0, 0, 0, 0,
    0,
    )

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

r = 0

try:
    # set target yaw
    r_controller.set_target(0)

    # control yaw
    while((time.time() - boot_time) < 30):
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'ATTITUDE':
            print(msg.yaw)
            r = r_controller.update(msg.yaw)
            print(r)
        send_manual_control(0,0,500,r)
        time.sleep(0.01)

    # Disarm
    time.sleep(3)
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')

except KeyboardInterrupt:
    # Disarm
    time.sleep(3)
    master.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    # Wait for disarm
    master.motors_disarmed_wait()
    print('Disarmed!')
