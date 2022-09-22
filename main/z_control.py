import numpy as np
import time
import math
import threading
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import matplotlib.pyplot as plt
from matplotlib import animation


def load_target_height():
    return 1050


class PID:

    def __init__(self):
        # Todo: testing & change to suitable value
        # PI > PID > PD > P
        self.Kp = 10
        self.Ki = 0.1
        self.Kd = 1
        self.max_output = 500
        # self.ITerm_max = 10
        self.delta_time = 0.2  # 5Hz

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


# test code
def draw_data(_):
    x.append(count)
    y.append(z)
    plt.cla()
    plt.scatter(x, y)
    plt.plot(x, y)


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


# test code

if __name__ == '__main__':

    # test code
    x, y = [], []
    count = 0
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
    DEPTH_HOLD = 'ALT_HOLD'
    DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
    while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
        master.set_mode(DEPTH_HOLD)
    print("set mode")

    global z
    z = 0
    print("Starting Thread")
    x = threading.Thread(target=thread_function, blocking=True)
    x.start()
    time.sleep(0.5)

    z_controller = PID()
    anima = animation.FuncAnimation(plt.gcf(), draw_data, interval=500)
    plt.show()

    ## start
    try:
        target_height = load_target_height()
        z_controller.set_target_height(target_height)
        z_controller.update(z)

        if z - target_height > 0:
            print(f'should>500, output: {500 + z_controller.output}')
        else:
            print(f'should<500, output: {500 + z_controller.output}')
        send_manual_control(0, 0, 500 - z_controller.output, 0)

    except KeyboardInterrupt:
        # Disarm
        send_manual_control(0, 0, 1000, 0)  # wait 3 sec to disarm
        print("Floating up")
        time.sleep(3)
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print('Disarmed!')
