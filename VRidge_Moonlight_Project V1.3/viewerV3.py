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

from filterpy.kalman import KalmanFilter
from anglewrapper import wrap

import pandas as pd


class imu_viewer():
    def __init__(self, width, height, hz, nav_frame):
        self.yawlist = [0]
        self.pitchlist = [0]
        self.rolllist = [0]

        self.outlist_y = [0]
        self.out_y = [0]
        self.out_y1 = [0]
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

    def start_t(self, imu):
        self.imu = imu
        self.thread = Thread(target=self.run, args=(self.imu, ))
        self.thread.start()

        self.plot()

    def run(self, imu):
        """
        Create IMU wireframe and keep display IMU information until it close

        :param IMU imu: imu object
        """
        self.wireframe = wireframe(imu.nav_frame)
        self.wireframe.initialize_cube(2, -2, 3, -3, 0.1, -0.1)

        alphaSOld_y = np.nan
        alphaSOld_p = np.nan
        alphaSOld_r = np.nan
        ry = 0
        rp = 0
        rr = 0

        pi = 3.141592653589793
        limit_angle = 180 * pi/180

        last_yaw = 0

        while self.alive:
            imu.update_noise_txt()

            # ---------------------------------------------------

            alphaS_y = self.yaw_ = imu.yaw
            alphaS_p = self.pitch_ = imu.pitch
            alphaS_r = self.roll_ = imu.roll
            # ---------------------------------------------------

            # ---------------------------------------------------

            # ---------------------------------------------------

            # ---------------------------------------------------
            if not np.isnan(alphaSOld_y):
                if alphaS_y - alphaSOld_y <= - pi:
                    ry += 1
                elif alphaS_y - alphaSOld_y >= pi:
                    ry -= 1
            self.alphaMy = alphaS_y + ry * 2 * pi

            if not np.isnan(alphaSOld_p):
                if alphaS_p - alphaSOld_p <= - pi:
                    rp += 1
                elif alphaS_p - alphaSOld_p >= pi:
                    rp -= 1
            self.alphaMp = alphaS_p + rp * 2 * pi

            if not np.isnan(alphaSOld_r):
                if alphaS_r - alphaSOld_r <= - pi:
                    rr += 1
                elif alphaS_r - alphaSOld_r >= pi:
                    rr -= 1
            self.alphaMr = alphaS_r + rr * 2 * pi

            # ---------------------------------------------------

            # self.yaw_ = (self.yaw_ + np.pi) % (2 * np.pi) - np.pi
            # print(self.yaw_)

            # ---------------------------------------------------
            self.yawlist.append(self.alphaMy)
            self.pitchlist.append(self.alphaMp)
            self.rolllist.append(self.alphaMr)

            # ---------------------------------------------------

            # ---------------------------------------------------

            lop = 10
            if len(self.yawlist) == lop:
                self.yawlist.pop(0)

            if len(self.pitchlist) == lop:
                self.pitchlist.pop(0)

            if len(self.rolllist) == lop:
                self.rolllist.pop(0)
            # ---------------------------------------------------

            # ---------------------------------------------------

            # ---------------------------------------------------

            # ---------------------------------------------------

            muy, covy, _, _ = fk.batch_filter(self.yawlist)
            My, Py, Cy, Ppy = fk.rts_smoother(muy, covy)

            a = np.array_split(My, 3, axis=1)

            # self.output_y = My[0][0]
            # self.y_1 = self.output_y

            mup, covp, _, _ = fk1.batch_filter(self.pitchlist)
            Mp, Pp, Cp, Ppp = fk1.rts_smoother(mup, covp)

            b = np.array_split(Mp, 3, axis=1)

            # self.output_p = Mp[0][0]
            # self.p_1 = self.output_p

            mur, covr, _, _ = fk2.batch_filter(self.rolllist)
            Mr, Pr, Cr, Ppr = fk2.rts_smoother(mur, covr)

            c = np.array_split(Mr, 3, axis=1)

            # self.output_r = Mr[0][0]
            # self.r_1 = self.output_r

            # ---------------------------------------------------

            # ---------------------------------------------------
            alphaSOld_y = alphaS_y
            alphaSOld_p = alphaS_p
            alphaSOld_r = alphaS_r
            # ---------------------------------------------------

            # ---------------------------------------------------
            # self.outlist_y.append(self.output_y)

            # if len(self.outlist_y) == 5:
            #     data_y = pd.Series(self.outlist_y)
            #     self.out_y1 = np.unwrap(data_y)
            #     print(self.out_y1)
            #     self.outlist_y.pop(0)

            # self.output_y = (self.output_y + np.pi) % (2 * np.pi) - np.pi
            # print(self.output_y)

            # ---------------------------------------------------
            print(len(a[0]),len(b[0]),len(c[0]))

            for i in range((len(a[0]))):
                self.output_y = float(a[0][i])

                self.output_p = float(b[0][i])

                self.output_r = float(c[0][i])
                

                self.output_y = (self.output_y + np.pi) % (2 * np.pi) - np.pi
                self.output_p = (self.output_p + np.pi) % (2 * np.pi) - np.pi
                self.output_r = (self.output_r + np.pi) % (2 * np.pi) - np.pi


                # print(self.output_y, self.output_p, self.output_r, i)

                # self.y_1 = self.output_y
                # self.p_1 = self.output_p
                # self.r_1 = self.output_r

                self.output_r_, self.output_p_, self.output_y_ = degrees(
                    self.output_r), degrees(self.output_p), degrees(self.output_y)

                self.output_rd, self.output_pd, self.output_yd = round(
                    self.output_r_, 5), round(self.output_p_, 5), round(self.output_y_, 5)

                euler = np.array(
                    [[self.output_rd], [self.output_pd], [self.output_yd]])

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
        plt.ylim(-3.5, 3.5)
        plt.xlim(0, 100)

        data1 = [0]
        data2 = [0]

        data3 = [0]
        data4 = [0]

        data5 = [0]
        data6 = [0]

        line1, = ax.plot(data1, label="yaw_predict")
        line2, = ax.plot(data2, label="yaw_raw")

        line3, = ax.plot(data3, label="pitch_predict")
        line4, = ax.plot(data4, label="pitch_raw")

        line5, = ax.plot(data5, label="roll_predict")
        line6, = ax.plot(data6, label="roll_raw")

        ax.set_xlabel("time")
        ax.set_ylabel("data")
        ax.legend()

        try:
            while self.alive:
                n_data1 = self.output_y
                n_data2 = self.yaw_

                n_data3 = self.output_p
                n_data4 = self.pitch_

                n_data5 = self.output_r
                n_data6 = self.roll_

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
                data6 = np.append(data6, n_data6)
                if len(data6) > 100:
                    data6 = data6[-100:]

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
                line6.set_ydata(data6)
                line6.set_xdata(np.arange(len(data6)))

                plt.draw()
                plt.pause(0.01)
        except KeyboardInterrupt:
            self.imu.out()
            sen.stop_sensor()
            print("out here")
            self.alive = False

        # pygame.quit()
        print("done for good")


def kalman_use(x, F, H, P, R, Q):
    kal_ = KalmanFilter(dim_x=2, dim_z=1)
    kal_.x = x
    kal_.F = F
    kal_.H = H
    kal_.P = P
    kal_.R = R
    kal_.Q = Q

    return kal_


if __name__ == '__main__':

    # url = 'ws://192.168.100.133:8090/sensors/connect?types=["android.sensor.accelerometer","android.sensor.gyroscope","android.sensor.magnetic_field"]'
    # url1 = 'ws://192.168.100.133:8090/sensors/connect?types="android.sensor.game_rotation_vector"'
    # # connect phone server from sensorfrom_phone library
    # sen = Sensor_phone(url=url)
    # sen.intialize()
    # sen.start_thread(True)

    # -----------------------------------------

    # fk = KalmanFilter(dim_x=2, dim_z=1)
    # fk.x = np.array([0., 1.])  # state (x and dx)
    # fk.F = np.array([[1., 1.], [0., 1.]])  # state transition matrix
    # fk.H = np.array([[1., 0.]])  # Measurement function
    # fk.P = 10.  # covariance matrix
    # fk.R = 7  # state uncertainty
    # fk.Q = 0.001  # process uncertainty

    # # -----------------------------------------
    # # -----------------------------------------

    # fk1 = KalmanFilter(dim_x=2, dim_z=1)
    # fk1.x = np.array([0., 1.])  # state (x and dx)
    # fk1.F = np.array([[1., 1.], [0., 1.]])  # state transition matrix
    # fk1.H = np.array([[1., 0.]])  # Measurement function
    # fk1.P = 10.  # covariance matrix
    # fk1.R = 9  # state uncertainty
    # fk1.Q = 0.001  # process uncertainty

    # -----------------------------------------
    # -----------------------------------------

    # fk2 = KalmanFilter(dim_x=2, dim_z=1)
    # fk2.x = np.array([0., 1.])  # state (x and dx)
    # fk2.F = np.array([[1., 1.], [0., 1.]])  # state transition matrix
    # fk2.H = np.array([[1., 0.]])  # Measurement function
    # fk2.P = 10.  # covariance matrix
    # fk2.R = 9  # state uncertainty
    # fk2.Q = 0.001  # process uncertainty

    # kal = KalmanFilter(dim_x=2, dim_z=1)
    x = np.array([0., 1.])  # state (x and dx)
    F = np.array([[1., 1.], [0., 1.]])  # state transition matrix
    H = np.array([[1.0, 0.]])  # Measurement function
    P = 100.  # covariance matrix
    R = 10  # state uncertainty
    Q = 0.001  # process uncertainty

    fk = kalman_use(x, F, H, P, R, Q)
    fk1 = kalman_use(x, F, H, P, R, Q)
    fk2 = kalman_use(x, F, H, P, R, Q)

    # -----------------------------------------
    # -----------------------------------------

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
    viewer.start_t(imu=imu)

    # pygame.quit()
