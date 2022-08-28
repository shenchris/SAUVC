import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import animation

import rospy
from sensor_msgs.msg import Imu


def listen_to_imu():
    rospy.init_node('listen_to_imu', anonymous=True)
    rospy.Subscriber("/vec6/sim_imu", Imu, localization)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


class KalmanFilter(object):
    def __init__(self, F=None, B=None, H=None, Q=None, R=None, P=None, x0=None):
        if (F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u=0):
        dt = 0.01
        self.F = np.array([[math.cos(self.x[7][0] * dt), -math.sin(self.x[7][0] * dt), 0,
                            dt * math.cos(dt * self.x[7][0]), dt * -math.sin(self.x[7][0] * dt), 0, 0,
                            -dt * (self.x[3][0] * dt + self.x[0][0]) * math.sin(dt * self.x[7][0]) + dt * (
                                    -self.x[4][0] * dt - self.x[1][0]) * math.cos(dt * self.x[7][0])],
                           [-math.sin(self.x[7][0] * dt), math.cos(self.x[7][0] * dt), 0,
                            -dt * math.sin(self.x[7][0] * dt), dt * math.cos(dt * self.x[7][0]), 0, 0,
                            dt * (-self.x[3][0] * dt - self.x[0][0]) * math.cos(dt * self.x[7][0]) - dt * (
                                    -self.x[4][0] * dt + self.x[1][0]) * math.sin(dt * self.x[7][0])],
                           [0, 0, 1, 0, 0, dt, 0, 0],
                           [0, 0, 0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, dt],
                           [0, 0, 0, 0, 0, 0, 0, 1]])
        self.x = np.dot(self.F, self.x)  # + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P),
                        (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)


def count_angle(msg):
    i = msg.orientation.x
    j = msg.orientation.y
    k = msg.orientation.z
    r = msg.orientation.w  # r:real
    # convert to rotation matrix
    R = np.array([[1 - 2 * (j * j + k * k), 2 * (i * j - k * r), 2 * (i * k + j * r)],
                  [2 * (i * j + k * r), 1 - 2 * (i * i + k * k), 2 * (j * k - i * r)],
                  [2 * (i * k - j * r), 2 * (j * k + i * r), 1 - 2 * (i * i + j * j)]])

    # apply on vector[0,1,0], which point to the head of boat
    unit_vector = np.array([[0.0],
                            [1.0],
                            [0.0]])
    R_vector = np.dot(R, unit_vector)

    # project to xy plane
    R_xy_vector = R_vector[0:2]

    # count angle
    x = R_xy_vector[0][0]
    y = R_xy_vector[1][0]
    _ = f"{x}+{y}j"
    angle = np.angle(eval(_), deg=True)
    if angle < 0:
        angle = angle + 360
    return angle


def localization(msg):
    global x, y, z, dt
    xa = msg.linear_acceleration.x
    ya = msg.linear_acceleration.y
    za = msg.linear_acceleration.z
    yaw = count_angle(msg)
    yaw_dot = msg.linear_acceleration.y

    kf.predict()
    load_data = np.array([[xa],
                  [ya],
                  [za],
                  [yaw],
                  [yaw_dot]], dtype="float64")
    kf.update(load_data)

    ### position
    x = x + kf.x[0] * dt + 0.5 * kf.x[3] * dt * dt
    y = y + kf.x[1] * dt + 0.5 * kf.x[4] * dt * dt
    z = z + kf.x[2] * dt + 0.5 * kf.x[5] * dt * dt



def main():
    global xa, ya, za, kf, x, y, z, dt
    x, y, z = 0, 0, 0

    dt = 0.01

    # X0 denoted initial System state
    X0 = np.array([[0],  # x_velocity (local)
                   [0],  # y_velocity
                   [0],  # z_velocity
                   [0],  # x_accel    (local)
                   [0],  # y_accel
                   [0],  # z_accel
                   [0],  # yaw x[6]
                   [0]], dtype="float64")  # yaw_dot x[7]

    # F denoted state transition jacobian matrix
    # F will be updated in iteration
    F = np.array([[1, 0, 0, dt, 0, 0, 0, 0],
                  [0, 1, 0, 0, dt, 0, 0, 0],
                  [0, 0, 1, 0, 0, dt, 0, 0],
                  [0, 0, 0, 1, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, dt],
                  [0, 0, 0, 0, 0, 0, 0, 1]], dtype="float64")
    # G denoted control matrix, don't have in this case

    # H denoted observation_matrix (x,y,z_accel,yaw,yaw_dot)
    H = np.array([[0, 0, 0, 1, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 1]], dtype="float64")

    # Q denoted transition_covariance
    # reference https://www.kalmanfilter.net/covextrap.html
    Qa = 10e-2 * np.array([[0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0, 0, 0],
                           [0, 0, 0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1]], dtype="float64")
    """
    Q = 10e-4 * np.array([[dt*dt, 0, 0, dt, 0, 0, 0, 0],
                           [0, dt*dt, 0, 0, dt, 0, 0, 0],
                           [0, 0, dt*dt, 0, 0, dt, 0, 0],
                           [dt, 0, 0, 1, 0, 0, 0, 0],
                           [0, dt, 0, 0, 0, 0, 0, 0],
                           [0, 0, dt, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1]])
    """
    Q = np.dot(np.dot(F, Qa), F.T)

    # Todo : change R based on imu accuracy
    # R denoted observation_covariance, should change based on imu accuracy
    R = np.array([[0.01, 0, 0, 0, 0],
                  [0, 0.01, 0, 0, 0],
                  [0, 0, 0.01, 0, 0],
                  [0, 0, 0, 0.01, 0],
                  [0, 0, 0, 0, 0.01]], dtype="float64")

    # P0 denoted initial_state_covariance
    P0 = np.array([[0.01, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0.01, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0.01, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0.1, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0.1, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0.1, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0.1, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0.1]], dtype="float64")

    kf = KalmanFilter(F=F, H=H, Q=Q, R=R, x0=X0, P=P0)

    try:
        listen_to_imu()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
