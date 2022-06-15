import numpy as np


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

    # reference: https://www.kalmanfilter.net/multiSummary.html
    def predict(self, u=0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    # reference: https://www.kalmanfilter.net/multiSummary.html
    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P),
                        (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)


# Todo : complete load imu
def load_imu():
    return np.array([[1],
                     [0],
                     [0]])


def main():
    ### Variables Setting Start ###

    # interval receive imu data
    dt = 0.01

    # X0 denoted initial System state
    # Question: is it better to read imu accel and put into init state?
    X0 = np.array([[0],  # x_position (global)
                   [0],  # y_position
                   [0],  # z_position
                   [1],  # x_velocity (global)
                   [0],  # y_velocity
                   [0],  # z_velocity
                   [0],  # x_accel    (global)
                   [0],  # y_accel
                   [0]])  # z_accel

    # F denoted state transition matrix
    F = np.array([[1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0, 0],
                  [0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0],
                  [0, 0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt],
                  [0, 0, 0, 1, 0, 0, dt, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0, dt, 0],
                  [0, 0, 0, 0, 0, 1, 0, 0, dt],
                  [0, 0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1]])
    # G denoted control matrix, don't have in this case

    # H denoted observation_matrix (x,y,z_accel)
    H = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1]])

    # Q denoted transition_covariance
    # Question: is it better to use DISCRETE NOISE MODEL or CONTINUOUS NOISE MODEL?
    # I set as DISCRETE NOISE MODEL in this case
    # If the dynamic model doesn't include a control input, we can project the random variance in acceleration
    # σa^2 on our dynamic model using the state transition matrix.
    # I set random variance in acceleration as 10e-4 based on other people code
    # reference https://www.kalmanfilter.net/covextrap.html
    Qa = 10e-4 * np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 1]])
    Q = np.dot(np.dot(F, Qa), F.T)

    # Todo : change R based on imu accuracy
    # R denoted observation_covariance, should change based on imu accuracy
    # Assume that the x,y,z measurements are uncorrelated
    R = np.array([[0.0002, 0, 0],
                  [0, 0.0002, 0],
                  [0, 0, 0.0002]])

    # P0 denoted initial_state_covariance
    # Question: is it better to read imu accel and set its value as initial_state_covariance?
    # I set velocity covariance as 10e-2 because I think it's impossible to be static in water
    # I set position covariance as 0 because the starting point is [0,0,0] in this case
    # I assume covariance in different variables are independent
    # reference https://www.kalmanfilter.net/multiExamples.html
    P0 = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 10e-2, 10e-6, 0, 0, 0, 0, 0, 0],
                      [0, 10e-6, 10e-4, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 10e-2, 10e-6, 0, 0, 0],
                      [0, 0, 0, 0, 10e-6, 10e-4, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 10e-2, 10e-6],
                      [0, 0, 0, 0, 0, 0, 0, 10e-6, 10e-4]])

    kf = KalmanFilter(F=F, H=H, Q=Q, R=R, x0=X0, P=P0)
    ### Variables Setting End ###

    while 1:
        kf.predict()
        # Todo : complete load imu
        z = load_imu()
        kf.update(z)

        # Todo : publish system state to Ros
        print(kf.x[0])

if __name__ == '__main__':
    main()
