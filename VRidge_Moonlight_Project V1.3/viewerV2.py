from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as tk
import pygame
import numpy as np
from utils.wireframe import wireframe
from operator import itemgetter
from collectV3 import collect
from ahrs import madgwick, mahony, ekf
import time
# from sensorfrom_phone import Sensor_phone
from sensorfrom_phoneUDP import Sensor_phoneUDP

from math import *
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread
from kalman_filter import KalmanFilter_

from filterpy.kalman import KalmanFilter


class imu_viewer():
    def __init__(self, width, height, hz, nav_frame):
        self.yawlist = []
        """
        Create window to display IMU information and visualize the IMU model

        :param int width: display window width
        :param int height: display window height
        :param int hz: window update rate
        :param str nav_frame: navigation frame
        """
        self.alive = True
        self.width = width
        self.height = height
        self.hz = hz
        self.nav_frame = nav_frame
        self.screen = pygame.display.set_mode((width, height))
        self.update_rate = pygame.time.Clock()
        pygame.font.init()
        self.font_size = 15
        self.font = pygame.font.SysFont('arial', self.font_size)
        # self.font = pygame.font.SysFont('Comic Sans MS', 15)
        self.background_color = (0, 0, 0)
        pygame.display.set_caption(
            'IMU attitude display with euler angle and quaternion')

        # Check parameter
        if (self.nav_frame != "ENU") and (self.nav_frame != "NED"):
            raise ValueError("Navigation frame should be either ENU or NED")
        if self.nav_frame == "ENU":
            self.rotation_seq = "zxy"
        elif self.nav_frame == "NED":
            self.rotation_seq = "zyx"

    def start_t(self, imu, kf, kf1, kf2):
        self.imu = imu
        self.thread = Thread(target=self.run, args=(self.imu, kf, kf1, kf2))
        self.thread.start()

        self.plot()

    def run(self, imu, kf, kf1, kf2):
        """
        Create IMU wireframe and keep display IMU information until it close

        :param IMU imu: imu object
        """
        self.wireframe = wireframe(imu.nav_frame)
        self.wireframe.initialize_cube(2, -2, 3, -3, 0.1, -0.1)

        alphaSOld = np.nan
        r = 0

        pi = 3.141592653589793
        limit_angle = 180 * pi/180

        while self.alive:
            # noise_g = imu.get_config_data("noise_g")
            # noise_a = imu.get_config_data("noise_a")
            # noise_m = imu.get_config_data("noise_m")
            # # print(noise_g,noise_a,noise_m)
            # imu.noise_chg([noise_g,noise_a,noise_m])
            imu.update_noise_txt()
            # print(imu.hz)

            # # ---------------------------------------------------
            # # Way to use kalman filter for one point
            # self.yaw_ = imu.yaw  # Data from sensor
            # if self.yaw_ >= 180: self.yaw_ = 180
            # if self.yaw_ <= -180: self.yaw_= -180

            # u_y = np.array([[self.yaw_]])
            # z_y = np.array([[self.yaw_]])

            # predicted_state_y = kf.predict(u_y)
            # updated_state_y = kf.update(z_y)

            # self.output_y = predicted_state_y[0][0]  # Output from filter
            # if self.output_y >= 180: self.output_y = 180
            # if self.output_y <= -180: self.output_y = -180
            # # self.nani = updated_state[0][0]
            # # print(imu.yaw, self.nani)
            # # time.sleep(0.0001)
            # # ---------------------------------------------------
            # # self.yaw_ = imu.yaw
            # alphaS = imu.yaw
            # if not np.isnan(alphaSOld):
            #     if alphaS - alphaSOld < - pi:
            #         r += 1
            #     elif alphaS - alphaSOld > pi:
            #         r -= 1
            # alphaM = alphaS + r * 2 * pi
            # self.yaw_ = alphaM

            # # if alphaM >= limit_angle: alphaM = limit_angle
            # # if alphaM <= -limit_angle: alphaM = -limit_angle
            # print(alphaM)
            # self.output_y = kf.kal_smoother(alphaM)
            # self.y_ = self.output_y

            # alphaSOld = alphaS
            # # ---------------------------------------------------

            self.yaw_ = imu.yaw
            reading = round(self.yaw_, 2)
            wut = self.yawlist.append(reading)
            print(wut)

            self.output_y = kf.kal_smoother(self.yaw_)
            self.y_ = self.output_y

            self.y_1 = 0

            # ---------------------------------------------------

            # # ---------------------------------------------------
            # # Way to use kalman filter for one point
            # self.pitch_ = imu.pitch  # Data from sensor

            # u_p = np.array([[self.pitch_]])
            # z_p = np.array([[self.pitch_]])

            # predicted_state_p = kf1.predict(u_p)
            # updated_state_p = kf1.update(z_p)

            # self.output_p = predicted_state_p[0][0]  # Output from filter
            # # self.nani = updated_state[0][0]
            # # print(imu.yaw, self.nani)
            # # time.sleep(0.0001)
            # ---------------------------------------------------
            self.pitch_ = imu.pitch
            self.output_p = kf1.kal_smoother(self.pitch_)
            self.p_ = self.output_p
            # ---------------------------------------------------

            self.roll_ = imu.roll
            self.output_r = kf2.kal_smoother(self.roll_)
            self.r_ = self.output_r

            self.output_r, self.output_p, self.output_y = degrees(
                self.output_r), degrees(self.output_p), degrees(self.output_y)

            self.output_rd, self.output_pd, self.output_yd = round(
                self.output_r, 5), round(self.output_p, 5), round(self.output_y, 5)

            euler = np.array(
                [[self.output_rd], [self.output_pd], [self.output_yd]])

            # print(euler)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.imu.out()
                    sen.stop_sensor()
                    print("out here")
                    self.alive = False
                    print('interrupted!1')
            self.update_rate.tick(self.hz)
            # self.display(imu.acc, imu.gyr, imu.mag, imu.euler,
            #              imu.quaternion, imu.body_frame, imu.nav_frame)
            self.display(imu.acc, imu.gyr, imu.mag, euler,
                         imu.quaternion, imu.body_frame, imu.nav_frame)
            pygame.display.update()
            # last_yaw = self.nani

        print("stop viewer")

    def display(self, acc, gyr, mag, euler, quaternion, body, nav):
        """
        Display IMU information and visualize the IMU model

        :param ndarray acc: accelerometer data
        :param ndarray gyr: gyroscope data
        :param ndarray mag: magnetometer data
        :param ndarray euler: euler angle
        :param ndarray quaternion: quaternion
        :param str body: body frame
        :param str nav: navigation frame
        """
        self.screen.fill(self.background_color)
        self.wireframe.update_attitude(euler)

        ax = round(acc[0][0], 2)
        ay = round(acc[1][0], 2)
        az = round(acc[2][0], 2)

        gx = round(gyr[0][0], 2)
        gy = round(gyr[1][0], 2)
        gz = round(gyr[2][0], 2)

        mx = round(mag[0][0], 2)
        my = round(mag[1][0], 2)
        mz = round(mag[2][0], 2)

        roll = round(euler[0][0], 2)
        pitch = round(euler[1][0], 2)
        yaw = round(euler[2][0], 2)

        w = round(quaternion[0][0], 2)
        x = round(quaternion[1][0], 2)
        y = round(quaternion[2][0], 2)
        z = round(quaternion[3][0], 2)

        # Display acceleration
        self.message_display("Acc (m/s^2): ", self.width * 0,
                             self.font_size * 0, (255, 255, 255))
        self.message_display("ax: {}".format(
            ax), self.width * 0, self.font_size * 1, (255, 255, 255))
        self.message_display("ay: {}".format(
            ay), self.width * 0, self.font_size * 2, (255, 255, 255))
        self.message_display("az: {}".format(
            az), self.width * 0, self.font_size * 3, (255, 255, 255))

        # Display angular velocity
        self.message_display("Gyro (rad/s): ", self.width *
                             0.1, self.font_size * 0, (255, 255, 255))
        self.message_display("gx: {}".format(
            gx), self.width * 0.1, self.font_size * 1, (255, 255, 255))
        self.message_display("gy: {}".format(
            gy), self.width * 0.1, self.font_size * 2, (255, 255, 255))
        self.message_display("gz: {}".format(
            gz), self.width * 0.1, self.font_size * 3, (255, 255, 255))

        # Display magnetic field
        self.message_display("Mag (µT): ", self.width * 0.2,
                             self.font_size * 0, (255, 255, 255))
        self.message_display("mx: {}".format(
            mx), self.width * 0.2, self.font_size * 1, (255, 255, 255))
        self.message_display("my: {}".format(
            my), self.width * 0.2, self.font_size * 2, (255, 255, 255))
        self.message_display("mz: {}".format(
            mz), self.width * 0.2, self.font_size * 3, (255, 255, 255))

        # Display body and navigation frame setting
        self.message_display("Body frame: {}".format(
            body), self.width * 0.3, self.font_size * 0, (255, 255, 255))
        self.message_display("Navigation frame: {}".format(
            nav), self.width * 0.3, self.font_size * 1, (255, 255, 255))

        # Display quaternion
        self.message_display("Quaternion: ", self.width *
                             0.75, self.font_size * 0, (255, 255, 255))
        self.message_display("w: {}".format(w), self.width *
                             0.75, self.font_size * 1, (255, 255, 255))
        self.message_display("x: {}".format(x), self.width *
                             0.75, self.font_size * 2, (255, 255, 255))
        self.message_display("y: {}".format(y), self.width *
                             0.75, self.font_size * 3, (255, 255, 255))
        self.message_display("z: {}".format(z), self.width *
                             0.75, self.font_size * 4, (255, 255, 255))

        # Display euler angle
        self.message_display("Euler Angles (Degree): ",
                             self.width * 0.85, self.font_size * 0, (255, 255, 255))
        if self.nav_frame == "ENU":  # zxy
            self.message_display("Roll: {} ({}-axis)".format(roll, list(self.rotation_seq)[
                                 1].upper()), self.width * 0.85, self.font_size * 1, (255, 255, 255))
            self.message_display("Pitch: {} ({}-axis)".format(pitch, list(self.rotation_seq)[
                                 2].upper()), self.width * 0.85, self.font_size * 2, (255, 255, 255))
            self.message_display("Yaw: {} ({}-axis)".format(yaw, list(self.rotation_seq)[
                                 0].upper()), self.width * 0.85, self.font_size * 3, (255, 255, 255))

        elif self.nav_frame == "NED":  # zyx
            self.message_display("Roll: {} ({}-axis)".format(roll, list(self.rotation_seq)[
                                 2].upper()), self.width * 0.85, self.font_size * 1, (255, 255, 255))
            self.message_display("Pitch: {} ({}-axis)".format(pitch, list(self.rotation_seq)[
                                 1].upper()), self.width * 0.85, self.font_size * 2, (255, 255, 255))
            self.message_display("Yaw: {} ({}-axis)".format(yaw, list(self.rotation_seq)[
                                 0].upper()), self.width * 0.85, self.font_size * 3, (255, 255, 255))

        # Display observation message
        self.message_display("Please observe the imu from top view",
                             self.width * 0.4, self.height * 0.95, (255, 255, 255))

        # Transform vertices to perspective view
        viewer_vertices = []
        viewer_depth = []
        for vertice in self.wireframe.vertices:
            point = np.array([[vertice.x], [vertice.y], [vertice.z]])
            new_point = self.wireframe.rotate_point(point)
            window_x, window_y = self.project2window(
                new_point.tolist()[0][0], new_point.tolist()[1][0], 70)
            viewer_vertices.append((window_x, window_y))
            viewer_depth.append(new_point.tolist()[2][0])

        # Calculate the average Z values of each face.
        avg_z = []
        for face in self.wireframe.faces:
            z = (viewer_depth[face.vertice_indexs[0]] + viewer_depth[face.vertice_indexs[1]] +
                 viewer_depth[face.vertice_indexs[2]] + viewer_depth[face.vertice_indexs[3]]) / 4.0
            avg_z.append(z)

        # Draw the faces using the Painter's algorithm:
        # sort the list according to avg_z
        for idx, val in sorted(enumerate(avg_z), key=itemgetter(1)):
            face = self.wireframe.faces[idx]
            pointList = [viewer_vertices[face.vertice_indexs[0]],
                         viewer_vertices[face.vertice_indexs[1]],
                         viewer_vertices[face.vertice_indexs[2]],
                         viewer_vertices[face.vertice_indexs[3]]]
            pygame.draw.polygon(self.screen, face.color, pointList)

    def message_display(self, text, x, y, text_color):
        """
        Display the color message in the window

        :param str text: display text
        :param int x: x-axis pixel
        :param int y: y-axis pixel
        :param tuple text_color: rgb color
        """
        text_surface = self.font.render(
            text, True, text_color, self.background_color)
        text_rect = text_surface.get_rect()
        text_rect.topleft = (x, y)
        self.screen.blit(text_surface, text_rect)

    def project2window(self, x, y, scaling_constant):
        """
        Project position into window coordinates pixel

        :param int x: x-axis position
        :param int y: y-axis position
        :param int scaling_constant: scale up or down the point

        :returns: 
            - window_x - sensor value in int16 format
            - window_y - sensor value in int16 format
        """
        window_x = round(x * scaling_constant + self.width / 2)
        window_y = round(y * scaling_constant + self.height / 2)
        return window_x, window_y

    def plot(self):
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plt.ylim(-4, 4)
        plt.xlim(0, 100)

        data1 = [0]
        data2 = [0]
        data3 = [0]
        data4 = [0]

        data5 = [0]

        line1, = ax.plot(data1, label="yaw_predict")
        line2, = ax.plot(data2, label="yaw_raw")

        line3, = ax.plot(data3, label="pitch_predict")
        line4, = ax.plot(data4, label="pitch_raw")

        line5, = ax.plot(data1, label="yaw_predict1")

        ax.set_xlabel("time")
        ax.set_ylabel("data")
        ax.legend()

        try:
            while self.alive:
                n_data1 = self.y_
                n_data2 = self.yaw_

                n_data3 = self.p_
                n_data4 = self.pitch_

                n_data5 = self.y_1

                # print(n_data1,n_data2)
                data1 = np.append(data1, n_data1)
                if len(data1) > 100:
                    data1 = data1[-100:]
                data2 = np.append(data2, n_data2)
                if len(data2) > 100:
                    data2 = data2[-100:]

                data3 = np.append(data3, n_data3)
                if len(data3) > 100:
                    data3 = data3[-100:]
                data4 = np.append(data4, n_data4)
                if len(data4) > 100:
                    data4 = data4[-100:]

                data5 = np.append(data5, n_data5)
                if len(data5) > 100:
                    data5 = data5[-100:]

                line1.set_ydata(data1)
                line1.set_xdata(np.arange(len(data1)))
                line2.set_ydata(data2)
                line2.set_xdata(np.arange(len(data2)))

                line3.set_ydata(data3)
                line3.set_xdata(np.arange(len(data3)))
                line4.set_ydata(data4)
                line4.set_xdata(np.arange(len(data4)))

                line5.set_ydata(data5)
                line5.set_xdata(np.arange(len(data5)))

                plt.draw()
                plt.pause(0.01)
        except KeyboardInterrupt:
            self.imu.out()
            sen.stop_sensor()
            print("out here")
            self.alive = False

        # pygame.quit()
        print("done for good")


if __name__ == '__main__':

    # url = 'ws://192.168.100.133:8090/sensors/connect?types=["android.sensor.accelerometer","android.sensor.gyroscope","android.sensor.magnetic_field"]'
    # url1 = 'ws://192.168.100.133:8090/sensors/connect?types="android.sensor.game_rotation_vector"'
    # # connect phone server from sensorfrom_phone library
    # sen = Sensor_phone(url=url)
    # sen.intialize()
    # sen.start_thread(True)

    dt = 1.0/60
    # F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
    # H = np.array([1, 0, 0]).reshape(1, 3)
    # Q = np.array([[10, 10, 0.0], [0.1, 0.1, 0.0], [0.0, 0.0, 0.0]])
    # R = np.array([20**2]).reshape(1, 1)

    F1 = np.array([[0.6, 0], [0, 0]])
    B1 = np.array([[0.3], [1]])
    H1 = np.array([[0.1, 0]])
    Q1 = np.array([[0.7, 0.5], [0, 0.1]])
    # R = np.array([[0.7]])
    R1 = np.array([0.5]).reshape(1, 1)

    # F = np.array([[0.8, 0], [0, 0]])
    # B = np.array([[0.5], [1]])
    # H = np.array([[0.7, 0]])
    # Q = np.array([[0.7, 0.6], [0, 1]])
    # R = np.array([[0.5]])

    # Initial state and covariance
    x01 = np.array([[0], [1]])
    P01 = np.array([[1, 0], [0, 1]])
    # Create Kalman Filter instance
    kf = KalmanFilter_(F=F1, B=B1, H=H1, Q=Q1, R=R1, x0=x01, P=P01)
    kf1 = KalmanFilter_(F=F1, B=B1, H=H1, Q=Q1, R=R1, x0=x01, P=P01)
    kf2 = KalmanFilter_(F=F1, B=B1, H=H1, Q=Q1, R=R1, x0=x01, P=P01)

    # -----------------------------------------

    fk = KalmanFilter(dim_x=2, dim_z=1)
    fk.x = np.array([0., 1.])  # state (x and dx)
    fk.F = np.array([[1., 1.], [0., 1.]])  # state transition matrix
    fk.H = np.array([[1., 0.]])  # Measurement function
    fk.P = 10.  # covariance matrix
    fk.R = 7.  # state uncertainty
    fk.Q = 0.001  # process uncertainty

    # -----------------------------------------
    address = ("0.0.0.0", 8080)
    sen = Sensor_phoneUDP(address=address)
    sen.intialize()
    sen.start_thread(True)

    window_width = 1080
    window_height = 720
    window_hz = 90
    nav_frame = "ENU"  # ENU/NED
    axis = 9
    calibration = False

    # ahrs = madgwick.Madgwick(axis, 10, nav_frame)
    # ahrs = mahony.Mahony(axis, 0.1, 0, nav_frame)
    # noise : list
    # gyroscope, accelerometer, magnetometer gaussian noise

    ahrs = ekf.EKF(axis, nav_frame)
    imu = collect(nav_frame=nav_frame, axis=axis, hz=window_hz, calibration=calibration,
                  load_calib=True, mag_sen=[0.8, 1, 0.8], degree=False)

    imu.initialization()
    imu.start_thread(ahrs=ahrs, sen=sen)
    viewer = imu_viewer(window_width, window_height, window_hz, nav_frame)
    viewer.start_t(imu=imu, kf=kf, kf1=kf1, kf2=kf2)

    # pygame.quit()
