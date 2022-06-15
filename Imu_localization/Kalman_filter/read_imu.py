import sys
import RTIMU
import os.path
import signal
import numpy
import time
import math

## setting
# imu calibration file
sys.path.append('.')
SETTINGS_FILE = "RTIMULib"
# sample size that will use to calculate drift
drift_sample_size = 300
# how many second will the starting data be ignored
ignore_initial_dirty_data_time = 2


def signal_handler(_, __):
    print('program terminated by Ctrl+C')
    sys.exit(0)


class IMUData:
    def __init__(self):
        # local frame
        self.x_accel_without_drift = 0
        self.y_accel_without_drift = 0
        self.z_accel_without_drift = 0

        # global frame
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.ns_accel = 0  # north-south direction
        self.ew_accel = 0  # east-west direction
        self.ud_accel = 0  # up-down direction


def init():
    # declare global variable
    global imu, drift_x, drift_y, drift_z, poll_interval

    # imu init
    print("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")

    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)

    print("IMU Name: " + imu.IMUName())

    if not imu.IMUInit():
        print("IMU Init Failed")
        sys.exit(1)
    else:
        print("IMU Init Succeeded")

    poll_interval = imu.IMUGetPollInterval()
    print("Recommended Poll Interval: %dmS\n" % poll_interval)

    # set fusion parameters
    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    # compute average on accel to obtain drift
    print("computing drift.... pls wait\n")

    sum_x, sum_y, sum_z, data_count = 0, 0, 0, 0
    while True:
        if imu.IMURead():
            data_init = imu.getIMUData()
            raw_x_init, raw_y_init, raw_z_init = data_init['accel']

            # ignore initial 2s dirty data from imu
            if (time.time() - startTime) > ignore_initial_dirty_data_time:
                if data_count < drift_sample_size:
                    sum_x = sum_x + raw_x_init
                    sum_y = sum_y + raw_y_init
                    sum_z = sum_z + raw_z_init
                    data_count = data_count + 1
                else:
                    break

            time.sleep(poll_interval * 1.0 / 1000.0)

    drift_x = sum_x / drift_sample_size
    drift_y = sum_y / drift_sample_size
    drift_z = sum_y / drift_sample_size

    print(f'the drift in raw x = {drift_x}')
    print(f'the drift in raw y = {drift_y}')
    print(f'the drift in raw z = {drift_z}\n')


def sendData(result):
    print(
        f'local frame accel x,y,z={result.x_accel_without_drift},{result.y_accel_without_drift},{result.z_accel_without_drift}')
    print(f'world frame accel x,y,z={result.ns_accel},{result.ew_accel},{result.ud_accel}')
    print(f'roll,pitch,yaw={result.roll},{result.pitch},{result.yaw}')


# Read IMU data
if __name__ == "__main__":
    # event handler for ctrl+c
    signal.signal(signal.SIGINT, signal_handler)

    startTime = time.time()
    global imu, drift_x, drift_y, drift_z, poll_interval
    init()
    result = IMUData()

    while True:
        if imu.IMURead():
            data = imu.getIMUData()
            raw_x, raw_y, raw_z = data['accel']
            roll, pitch, yaw = data['fusionPose']

            # deal with drift
            raw_x_without_drift = raw_x - drift_x
            raw_y_without_drift = raw_y - drift_y
            raw_z_without_drift = raw_z - drift_z
            result.x_accel_without_drift = raw_x_without_drift
            result.y_accel_without_drift = raw_y_without_drift
            result.z_accel_without_drift = raw_z_without_drift

            # map to world frame
            rotMatrix = numpy.array(
                [[math.cos(yaw) * math.cos(pitch),
                  (-1) * math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.sin(yaw) * math.cos(roll),
                  (-1) * math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll)],
                 [math.sin(yaw) * math.cos(pitch),
                  (-1) * math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll),
                  (-1) * math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll)],
                 [math.sin(pitch), math.cos(pitch) * math.sin(roll), math.cos(pitch) * math.sin(roll)]])

            data_array = numpy.array([raw_x_without_drift, raw_y_without_drift, raw_z_without_drift])
            result_array = numpy.dot(rotMatrix, data_array)

            result.ns_accel = result_array[0]
            result.ew_accel = result_array[1]
            result.ud_accel = result_array[2]

            # Todo: use ROS to publish result
            sendData(result)

            time.sleep(poll_interval * 1.0 / 1000.0)
