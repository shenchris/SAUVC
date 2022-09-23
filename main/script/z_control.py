import numpy as np
import time
import math
import threading
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import csv


def load_target_height():
    return 1050


class PID:

    def __init__(self):
        # Todo: testing & change to suitable value
        # PI > PID > PD > P
        self.Kp = 100
        self.Ki = 10
        self.Kd = 1
        self.max_output = 500
        # self.ITerm_max = 10
        self.delta_time = 1.4

        # init
        self.target_height = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.output = 0.0

    def clear(self):
        self.target_height = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.output = 0.0

    # Calculate P, I ,D , and output thruster command
    def update(self, feedback_value):
        error = self.target_height - feedback_value
        delta_time = self.delta_time
        delta_error = error - self.last_error

        self.PTerm = self.Kp * error
        self.ITerm += error * delta_time

        self.DTerm = delta_error / delta_time

        # Remember last time and last error for next calculation
        self.last_error = error
        self.output = np.clip(self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm), -self.max_output,
                              self.max_output)

    def set_target_height(self, goal):
        self.target_height = goal


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
    global z, count
    while True:
        try:
            z = master.recv_match(type='SCALED_PRESSURE2', blocking=True).to_dict()["press_abs"]
            count += 1
        except KeyboardInterrupt:
            master.arducopter_disarm()
            master.motors_disarmed_wait()
            print('Disarmed!')


def set_target_depth(depth):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    """
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=(  # ignore everything except z position
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), lat_int=0, lon_int=0, alt=depth,  # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0,  # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )


# test code

if __name__ == '__main__':

    # test code
    x, y = [], []
    count = 0
    #f = open('pressure_value', 'w')
    #writer = csv.writer(f)
    # test code

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
    DEPTH_HOLD = 'STABILIZE'
    DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
    while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
        master.set_mode(DEPTH_HOLD)
    print("set STABILIZE")

    global z
    z = 0
    print("Starting Thread")
    x = threading.Thread(target=thread_function)
    x.start()
    time.sleep(0.5)

    z_controller = PID()

    ## start
    try:
        t = time.time()
        print("PID")
        while time.time() - t < 20:

            target_height = load_target_height()
            z_controller.set_target_height(target_height)
            z_controller.update(z)
            #writer.writerow(z)

            if z - target_height > 0:
                print(f'should>500, output: {500 - z_controller.output}')
            else:
                print(f'should<500, output: {500 - z_controller.output}')
            send_manual_control(0, 0, 500 - z_controller.output, 0)

        print("Floating up")
        send_manual_control(0, 0, 1000, 0)  # wait 3 sec to disarm
        time.sleep(3)
        send_manual_control(0, 0, 500, 0)
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print('Disarmed!')


    except KeyboardInterrupt:
        # Disarm
        send_manual_control(0, 0, 1000, 0)  # wait 3 sec to disarm
        #f.close()
        print("Floating up")
        time.sleep(3)
        send_manual_control(0, 0, 500, 0)
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print('Disarmed!')
        #f.close()
