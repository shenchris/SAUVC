import time
import math
import threading
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

target_height = 1050

# motion for testing
x, y, z, r = 0, 1000, 500, 0

sleep = 3


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


def set_target_attitude(roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0  # roll rate, pitch rate, yaw rate, thrust
    )


def send_manual_control(x, y, z, r):
    master.mav.manual_control_send(
        master.target_system,
        x,  # -1000 to 1000, static=0, backward<0, forward>0
        y,  # -1000 to 1000, static=0, left<0, right>0
        z,  # 0 to 1000, static=500, downward<500, upward>500
        r,  # -1000 to 1000, static=0, anti-clockwise<0, clockwise>0
        0  # useless (for other purpose)
    )


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


def thread_function():
    global pressure, yaw
    while True:
        try:
            msg = master.recv_match(type=['SCALED_PRESSURE2', 'ATTITUDE'], blocking=True).to_dict()
            if msg['mavpackettype'] == 'SCALED_PRESSURE2':
                pressure = msg['press_abs']
            elif msg['mavpackettype'] == 'ATTITUDE':
                yaw = msg['yaw']
        except KeyboardInterrupt:
            master.arducopter_disarm()
            master.motors_disarmed_wait()


## start
global pressure, yaw
pressure = None
yaw = None
x = threading.Thread(target=thread_function, blocking=True)
x.start()
time.sleep(0.5)

try:
    target_yaw = yaw
    while target_yaw != 0:
        print(f'set_target_attitude: {target_yaw}')
        set_target_attitude(0, 0, target_yaw)
        # send_manual_control(0, 0, 500, 0)
        time.sleep(sleep)

        z_controller = PID()
        t = time.time()
        while (time.time() - t) < 5:
            try:
                z_controller.set_target_height(target_height)
                z_controller.update(pressure)

                if z - target_height > 0:
                    print(f'should>500, output: {500 - z_controller.output}')
                else:
                    print(f'should<500, output: {500 - z_controller.output}')
                send_manual_control(0, 0, 500 - z_controller.output, 0)

            except KeyboardInterrupt:
                # Disarm
                send_manual_control(0, 0, 1000, 0)  # wait 3 sec to disarm
                print("Floating up")
                time.sleep(3)
                master.arducopter_disarm()
                master.motors_disarmed_wait()
                print('Disarmed!')

        for i in range(5):
            send_manual_control(x, y, z, r)
            print("moving")
            time.sleep(1)

except KeyboardInterrupt:
    # Disarm
    send_manual_control(0, 0, 1000, 0)  # wait 3 sec to disarm
    time.sleep(3)
    print("Floating up")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print('Disarmed!')

# clean up (disarm) at the end
send_manual_control(0, 0, 1000, 0)  # wait 3 sec to disarm
time.sleep(3)
print("Floating up")
master.arducopter_disarm()
master.motors_disarmed_wait()
print('Disarmed!')
