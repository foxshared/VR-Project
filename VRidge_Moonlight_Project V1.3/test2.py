import numpy as np
import matplotlib.pyplot as plt


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
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
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


def example():
    dt = 1.0/60
    # F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
    # H = np.array([1, 0, 0]).reshape(1, 3)
    # Q = np.array([[10, 10, 0.0], [0.1, 0.1, 0.0], [0.0, 0.0, 0.0]])
    # R = np.array([20**2]).reshape(1, 1)

    # F = np.array([[0.8, 0], [0, 0]])
    # B = np.array([[0.5], [0.4]])
    # H = np.array([[0.7, 0]])
    # Q = np.array([[0.7, 0], [0, 1]])
    # R = np.array([[0.45]])

    F = np.array([[0.8, 0], [0, 0]])
    B = np.array([[0.3], [1]])
    H = np.array([[0.7, 0]])
    Q = np.array([[0.7, 0.9], [0, 1]])
    R = np.array([[0.5]])

    # F = np.array([[0.8, 0], [0, 0]])
    # B = np.array([[0.5], [1]])
    # H = np.array([[0.7, 0]])
    # Q = np.array([[1, 0], [0, 1]])
    # R = np.array([[0.1]])
    # Initial state and covari

    x0 = np.array([[0], [1]])
    P0 = np.array([[0.1, 0], [0, 1]])
    # Create Kalman Filter instance

    x = np.linspace(-10, 10, 120)
    measurements = - (x**2 + 2*x - 2) + np.random.normal(0, 2, 120)

    kf = KalmanFilter(F=F, H=H, Q=Q, R=R, x0=x0, P=P0)
    predictions = []

    for z in measurements:

        pre = kf.predict()
        # pr_= np.dot(H, pre)[0]
        pr_ = pre[0][0]
        predictions.append(pr_)
        kf.update(z)

        # print(z,pre[0],pr_)

    print(predictions)
    plt.plot(range(len(measurements)), measurements, label='Measurements')
    plt.plot(range(len(predictions)), np.array(
        predictions), label='Kalman Filter Prediction')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    example()
