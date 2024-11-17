# Calibrated sensor

import time
import numpy as np
from progress.bar import Bar
from utils.transformation import ENU2NED, NED2ENU, acc2eul, acc2quat, accmag2eul, accmag2quat
from utils.orientation import quat2eul
import math
from threading import Thread
import yaml
import sys
from configparser import ConfigParser
import os


# standard acceleration of gravity
g = 9.80665


class collect():
    def __init__(self, nav_frame="", axis=9, hz=100, calibration=False, load_calib=False, mag_sen=[1, 1, 1],degree = True):
        self.degree = degree
        self.ConfigParser = ConfigParser()
        self.create_config()
        self.noise = [0.1, 0.1, 0.1]
        self.wa, self.xa, self.ya, self.za = 0, 0, 0, 0

        self.alive = True
        # Magnetometer parameter
        self.mag_bias = np.zeros((3, 1))
        self.mag_scale = np.ones((3, 1))
        self.mag_misalignment = np.zeros((6, 1))
        self.mag_strength = 0

        # Accelerometer and gyroscope parameter
        self.accel_bias = np.zeros((3, 1))
        self.accel_scale = np.ones((3, 1))
        self.accel_misalignment = np.zeros((6, 1))
        self.gyro_bias = np.zeros((3, 1))
        self.gyro_scale = np.ones((3, 1))
        self.gyro_misalignment = np.zeros((6, 1))

        # driver parameter
        self.nav_frame = nav_frame  # original navigation frame of MPU6500 is ENU
        self.hz = hz
        self.axis = axis
        self.dt = 1/self.hz
        self.queue_size = 20
        self.window_size = 5
        self.gyro_queue = np.empty([1, 3])
        self.calibration = calibration

        # Check parameter
        if (self.axis != 6) and (self.axis != 9):
            raise ValueError("Axis must be 6 or 9")
        if self.nav_frame == "NED":
            self.body_frame = "FRD"
            self.rotation_seq = "zyx"
            # self.rotation_seq = "xyz"
        elif self.nav_frame == "ENU":
            self.body_frame = "FRD"
            self.rotation_seq = "zxy"
            print(self.rotation_seq)

        self.acc_raw = [0, 0, 0]  # xyz
        self.gyro_raw = [0, 0, 0]  # xyz
        self.mag_raw = [0, 0, 0]  # xyz
        self.game_orientation = []

        # self.adjustment_x = (((mag_sen[0]-128)*0.5/128)+1)
        # self.adjustment_y = (((mag_sen[1]-128)*0.5/128)+1)
        # self.adjustment_z = (((mag_sen[2]-128)*0.5/128)+1)

        self.adjustment_x = mag_sen[0]
        self.adjustment_y = mag_sen[1]
        self.adjustment_z = mag_sen[2]

        # Load old config from yaml file
        if self.calibration == False and load_calib == True:
            f = open("config.yaml", "r")
            self.config = yaml.load(f, Loader=yaml.FullLoader)
            gyro_bias = ["gx_bias", "gy_bias", "gz_bias"]
            gyro_scale = ["gx_scale", "gy_scale", "gz_scale"]
            # gyro_misalignment = ["gyro_xy_mis", "gyro_xz_mis",
            #                      "gyro_yx_mis", "gyro_yz_mis", "gyro_zx_mis", "gyro_zy_mis"]
            accel_bias = ["ax_bias", "ay_bias", "az_bias"]
            accel_scale = ["ax_scale", "ay_scale", "az_scale"]
            # accel_misalignment = ["accel_xy_mis", "accel_xz_mis",
            #                       "accel_yx_mis", "accel_yz_mis", "accel_zx_mis", "accel_zy_mis"]
            mag_bias = ["mx_bias", "my_bias", "mz_bias"]
            mag_scale = ["mx_scale", "my_scale", "mz_scale"]
            # mag_misalignment = ["mag_xy_mis", "mag_xz_mis",
            #                     "mag_yx_mis", "mag_yz_mis", "mag_zx_mis", "mag_zy_mis"]

            for i, element in enumerate(gyro_bias):
                self.gyro_bias[i][0] = self.config[nav_frame][element]
            for i, element in enumerate(gyro_scale):
                self.gyro_scale[i][0] = self.config[nav_frame][element]
            # for i, element in enumerate(gyro_misalignment):
            #     self.gyro_misalignment[i][0] = self.config[nav_frame][element]
            for i, element in enumerate(accel_bias):
                self.accel_bias[i][0] = self.config[nav_frame][element]
            for i, element in enumerate(accel_scale):
                self.accel_scale[i][0] = self.config[nav_frame][element]
            # for i, element in enumerate(accel_misalignment):
            #     self.accel_misalignment[i][0] = self.config[nav_frame][element]
            for i, element in enumerate(mag_bias):
                self.mag_bias[i][0] = self.config[nav_frame][element]
            for i, element in enumerate(mag_scale):
                self.mag_scale[i][0] = self.config[nav_frame][element]
            # for i, element in enumerate(mag_misalignment):
            #     self.mag_misalignment[i][0] = self.config[nav_frame][element]
            print("done load calib")

    def gyro_calibration(self, s: int):
        """
        Calculate the gyroscope bias to calibrate the gyroscope

        :param int s: time for calibration
        :returns: 
            - gyro_scale (ndarray) - 3-axis gyroscope scale
            - gyro_bias (ndarray) - 3-axis gyroscope bias
            - gyro_misalignment (ndarray) - 3-axis gyroscope misalignment
        """
        if s > 0 and self.calibration == True:
            print('Start gyroscope calibration - Do not move the IMU for {}s'.format(s))
            total_gyro = np.zeros((3, 1))

            with Bar('Processing... ', max=int(s*self.hz)) as bar:
                for i in range(s*self.hz):
                    gx, gy, gz = self.get_gyro()
                    total_gyro += np.array([[gx], [gy], [gz]])
                    bar.next()
                    time.sleep(1/self.hz)

            # calculate bias
            self.gyro_bias = total_gyro/(s*self.hz)

            # calculate scale
            self.gyro_scale = np.array([[1], [1], [1]])

            # calculate misalignment
            self.gyro_misalignment = np.array([[0], [0], [0], [0], [0], [0]])

            print("Finish gyroscope calibration")
            print(self.gyro_scale, self.gyro_bias, self.gyro_misalignment)
        return self.gyro_scale, self.gyro_bias, self.gyro_misalignment

    def accel_calibration(self, s: int):
        """
        Calculate the accelerometer bias, scale, misalignment with six calibration measurements \n
        Using least square method to solve the error

        :param int s: time for calibration
        :returns: 
            - accel_scale (ndarray) - 3-axis accelerometer scale
            - accel_bias (ndarray) - 3-axis accelerometer bias
            - accel_misalignment (ndarray) - 3-axis accelerometer misalignment

        .. Reference
        .. [1] 'Accelerometer calibration <https://zhuanlan.zhihu.com/p/296381805>'
        .. [2] 'Least square prove <https://zhuanlan.zhihu.com/p/87582571>'
        """
        if s > 0 and self.calibration == True:
            order = ["x", "y", "z", "-x", "-y", "-z"]
            calibration = []
            for i in range(6):
                input('Place IMU {} axis ({}) pointing downward and do not move the IMU for {}s'.format(
                    order[i], self.nav_frame, s))
                total_accel = np.zeros((3))

                with Bar('IMU {} axis ({}) Processing... '.format(order[i], self.nav_frame), max=int(s*self.hz)) as bar:
                    for j in range(s*self.hz):
                        ax, ay, az = self.get_accel()
                        total_accel += np.array([ax, ay, az])
                        bar.next()
                        time.sleep(1/self.hz)
                avg_accel = total_accel/(s*self.hz)
                calibration.append(avg_accel.tolist())
            calibration = np.array(calibration)
            calibration = np.append(calibration, np.ones((6, 1))*-1, axis=1)
            positive = np.diag(np.full(3, g))
            negative = np.diag(np.full(3, -g))
            if self.nav_frame == "ENU":
                target = np.vstack((negative, positive))
            elif self.nav_frame == "NED":
                target = np.vstack((positive, negative))
            error_matrix = np.linalg.inv(
                calibration.T @ calibration) @ calibration.T @ target

            # calculate bias
            x_bias = error_matrix[3][0]
            y_bias = error_matrix[3][1]
            z_bias = error_matrix[3][2]
            self.accel_bias = np.array([[x_bias], [y_bias], [z_bias]])

            # calculate scale
            x_scale = error_matrix[0][0]
            y_scale = error_matrix[1][1]
            z_scale = error_matrix[2][2]
            self.accel_scale = np.array([[x_scale], [y_scale], [z_scale]])

            # calculate misalignment
            xy_mis = error_matrix[0][1]
            xz_mis = error_matrix[0][2]
            yx_mis = error_matrix[1][0]
            yz_mis = error_matrix[1][2]
            zx_mis = error_matrix[2][0]
            zy_mis = error_matrix[2][1]
            self.accel_misalignment = np.array(
                [[xy_mis], [xz_mis], [yx_mis], [yz_mis], [zx_mis], [zy_mis]])
            print("Finish accelerometer calibration")
            print(self.accel_scale, self.accel_bias, self.accel_misalignment)
        return self.accel_scale, self.accel_bias, self.accel_misalignment

    def mag_calibration(self, s: int):
        """
        Calculate the magnetometer bias, scale, misalignment, geomagnetic field strength with four element calibration
        Using least square method to solve the error

        :param int s: time for calibration
        :returns: 
            - mag_scale (ndarray) - 3-axis magnetometer scale (soft-iron offset)
            - mag_bias (ndarray) - 3-axis magnetometer bias (hard-iron offset)
            - mag_misalignment (ndarray) - 3-axis magnetometer misalignment (soft-iron offset)
            - mag_strength (float) - geomagnetic field strength in µT

        .. Reference
        .. [1] 'four element calibration <https://www.nxp.com/docs/en/application-note/AN5019.pdf>'
        .. [2] 'ten element calibration <https://github.com/nliaudat/magnetometer_calibration/blob/main/calibrate.py>'
        """
        if s > 0 and self.calibration == True:
            input("Please move the IMU in slow motion in all possible directions, the calibration process takes {}s".format(s))
            calibration = []
            target = []
            with Bar('Processing... ', max=int(s*self.hz)) as bar:
                for i in range(s*self.hz):
                    mx, my, mz = self.get_mag()
                    calibration.append([mx, my, mz, 1])
                    target.append([mx**2+my**2+mz**2])
                    bar.next()
                    time.sleep(1/self.hz)
            calibration = np.array(calibration)
            target = np.array(target)

            error_matrix = np.linalg.inv(
                calibration.T @ calibration) @ calibration.T @ target

            # calculate bias
            x_bias = 0.5*error_matrix[0][0]
            y_bias = 0.5*error_matrix[1][0]
            z_bias = 0.5*error_matrix[2][0]
            self.mag_bias = np.array([[x_bias], [y_bias], [z_bias]])

            # calculate scale
            self.mag_scale = np.array([[1], [1], [1]])

            # calculate misalignment
            self.mag_misalignment = np.array([[0], [0], [0], [0], [0], [0]])

            # calculate strength
            strength = (error_matrix[3][0]+x_bias**2+y_bias**2+z_bias**2)**0.5
            self.mag_strength = np.array([[strength]])

            print(self.mag_scale, self.mag_bias,
                  self.mag_misalignment, self.mag_strength)
        return self.mag_scale, self.mag_bias, self.mag_misalignment, self.mag_strength

    def get_gyro(self):  # ENU
        """
        MPU6500 gyroscope data in right hand coordinates (rad/s)

        ENU: \n
        gx is positive when rotate clockwise along x-axis \n
        gy is positive when rotate clockwise along y-axis \n
        gz is positive when rotate anti-clockwise along z-axis \n

        NED: \n
        gx is positive when rotate clockwise along x-axis \n
        gy is positive when rotate clockwise along y-axis \n
        gz is positive when rotate clockwise along z-axis \n

        :returns: 
            - gx (float) - x-axis gyroscope data in rad/s
            - gy (float) - y-axis gyroscope data in rad/s
            - gz (float) - z-axis gyroscope data in rad/s
        """
        try:
            # gx = self.gyro_raw[0]*math.pi/180
            # gy = self.gyro_raw[1]*math.pi/180
            # gz = self.gyro_raw[2]*math.pi/180

            # gx = self.gyro_raw[0]
            # gy = self.gyro_raw[1]
            # gz = self.gyro_raw[2]

            gx = self.gy_
            gy = self.gx_
            gz = self.gz_

            # gx = self.gy_*math.pi/180
            # gy = self.gx_*math.pi/180
            # gz = self.gz_*math.pi/180
        except:
            raise ConnectionError("I2C Connection Failure")

        # convert to NED frame
        if self.nav_frame == "NED":
            gx, gy, gz = ENU2NED(gx, gy, gz)

        # gyroscope model: calibrated measurement = (matrix)*(raw measurement - bias)
        x_scale = self.gyro_scale[0][0]
        y_scale = self.gyro_scale[1][0]
        z_scale = self.gyro_scale[2][0]
        x_bias = self.gyro_bias[0][0]
        y_bias = self.gyro_bias[1][0]
        z_bias = self.gyro_bias[2][0]
        xy_mis = self.gyro_misalignment[0][0]
        xz_mis = self.gyro_misalignment[1][0]
        yx_mis = self.gyro_misalignment[2][0]
        yz_mis = self.gyro_misalignment[3][0]
        zx_mis = self.gyro_misalignment[4][0]
        zy_mis = self.gyro_misalignment[5][0]
        if self.calibration == False:
            gx = (x_scale * gx + xy_mis * gy + xz_mis * gz) - x_bias
            gy = (yx_mis * gx + y_scale * gy + yz_mis * gz) - y_bias
            gz = (zx_mis * gx + zy_mis * gy + z_scale * gz) - z_bias

        return gx, gy, gz

    def get_accel(self):  # ENU
        """
        MPU6500 accelerometer data in Earth's reference (m/s^2)
        Accelerometer channel is negative when pointing up and aligned against gravity

        ENU: \n
        Gravity is defined as negative when pointing upward \n
        ax = +9.80665 m/s^2 when the right hand side pointing upward \n
        ay = +9.80665 m/s^2 when front side pointing upward \n
        az = +9.80665 m/s^2 when upper side pointing upward \n

        NED:
        Gravity is defined as negative when pointing downward \n
        ax = +9.80665 m/s^2 when front side pointing downward \n
        ay = +9.80665 m/s^2 when the right hand side pointing downward \n
        az = +9.80665 m/s^2 when under side pointing downward \n

        :returns: 
            - ax (float) - x-axis accelerometer data in m/s^2
            - ay (float) - y-axis accelerometer data in m/s^2
            - az (float) - z-axis accelerometer data in m/s^2
        """
        try:
            # ax = self.acc_raw[0]
            # ay = self.acc_raw[1]
            # az = self.acc_raw[2]

            ax = self.ax_
            ay = self.ay_
            az = self.az_
        except:
            raise ConnectionError("I2C Connection Failure")

        # convert to m/s^2
        ax = ax*g
        ay = ay*g
        az = az*g

        # convert to NED frame
        if self.nav_frame == "NED":
            # ax = ax*-1
            # ay = ay*-1
            # az = az*-1
            ax, ay, az = ENU2NED(ax, ay, az)

        # accelerometer model: calibrated measurement = (matrix)*(raw measurement - bias)
        x_scale = self.accel_scale[0][0]
        y_scale = self.accel_scale[1][0]
        z_scale = self.accel_scale[2][0]
        x_bias = self.accel_bias[0][0]
        y_bias = self.accel_bias[1][0]
        z_bias = self.accel_bias[2][0]
        xy_mis = self.accel_misalignment[0][0]
        xz_mis = self.accel_misalignment[1][0]
        yx_mis = self.accel_misalignment[2][0]
        yz_mis = self.accel_misalignment[3][0]
        zx_mis = self.accel_misalignment[4][0]
        zy_mis = self.accel_misalignment[5][0]
        if self.calibration == False:
            ax = (x_scale * ax + xy_mis * ay + xz_mis * az) - x_bias
            ay = (yx_mis * ax + y_scale * ay + yz_mis * az) - y_bias
            az = (zx_mis * ax + zy_mis * ay + z_scale * az) - z_bias
        return ax, ay, az

    def get_mag(self):  # ENU
        """
        AK8963 magnetometer data in Earth's reference (µT)

        ENU: \n
        mx is positive when the right hand side pointing to north \n
        my is positive when the front side pointing to north \n
        mz is positive when the upper side pointing to north \n

        NED: \n
        mx is positive when the front side pointing to north \n
        my is positive when the right hand side pointing to north \n
        mz is positive when the under side pointing to north \n

        :returns: 
            - mx (float) - x-axis magnetometer data in µT
            - my (float) - y-axis magnetometer data in µT
            - mz (float) - z-axis magnetometer data in µT
        """
        try:
            # mx = self.mag_raw[0]
            # my = self.mag_raw[1]
            # mz = self.mag_raw[2]

            # mx = self.mx_
            # my = self.my_
            # mz = self.mz_

            # mx = self.mx_*self.adjustment_x
            # my = self.my_*self.adjustment_y
            # mz = self.mz_*self.adjustment_z

            mx = self.mx_*self.adjustment_x
            my = self.my_*self.adjustment_y
            mz = self.mz_*self.adjustment_z

            # # print("x {} y {} z {} -------------- x {} y {} z {}".format(self.mx_,self.my_,self.mz_,mx,my,mz))
            # delete_last_line()
        except:
            raise ConnectionError("I2C Connection Failure")

        # if self.nav_frame == "NED":
        #     mx, my, mz = ENU2NED(mx, my, mz)

        # magnetometer model: calibrated measurement = (matrix)*(raw measurement - bias)
        x_scale = self.mag_scale[0][0]
        y_scale = self.mag_scale[1][0]
        z_scale = self.mag_scale[2][0]
        x_bias = self.mag_bias[0][0]
        y_bias = self.mag_bias[1][0]
        z_bias = self.mag_bias[2][0]
        xy_mis = self.mag_misalignment[0][0]
        xz_mis = self.mag_misalignment[1][0]
        yx_mis = self.mag_misalignment[2][0]
        yz_mis = self.mag_misalignment[3][0]
        zx_mis = self.mag_misalignment[4][0]
        zy_mis = self.mag_misalignment[5][0]
        if self.calibration == False:
            mx = (x_scale * mx + xy_mis * my + xz_mis * mz) - x_bias
            my = (yx_mis * mx + y_scale * my + yz_mis * mz) - y_bias
            mz = (zx_mis * mx + zy_mis * my + z_scale * mz) - z_bias
        return mx, my, mz

    def get_gyro1_(self):
        # yxz
        gy = self.gyro_raw[0]
        gx = self.gyro_raw[1]
        gz = self.gyro_raw[2]

        return gx, gy, gz

    def get_accel1_(self):
        ay = self.acc_raw[0]
        ax = self.acc_raw[1]
        az = self.acc_raw[2]

        return ax, ay, az

    def get_mag1_(self):

        my = self.mag_raw[0]
        mx = self.mag_raw[1]
        mz = self.mag_raw[2]

        return mx, my, mz

    def get_euler(self):
        """
        MPU9250 Euler angle

        :returns: 
            - roll (float) - x-axis Euler angle in degree
            - pitch (float) - y-axis Euler angle in degree
            - yaw (float) - z-axis Euler angle in degree
        """
        if self.axis == 6:
            self.roll, self.pitch, self.yaw = acc2eul(
                self.ax, self.ay, self.az, nav=self.nav_frame)
        elif self.axis == 9:
            self.roll, self.pitch, self.yaw = accmag2eul(
                self.ax, self.ay, self.az, self.mx, self.my, self.mz, nav=self.nav_frame)
        
        if self.degree:
            self.roll, self.pitch, self.yaw = math.degrees(
                self.roll), math.degrees(self.pitch), math.degrees(self.yaw)

        # self.euler = np.array([[0], [0], [self.yaw]])
        self.euler = np.array([[self.roll], [self.pitch], [self.yaw]])
        # self.euler = np.array([[0], [self.pitch], [self.yaw]])
        return self.roll, self.pitch, self.yaw

    def get_quaternion(self):
        """
        MPU9250 Quaternion

        :returns: 
            - w (float) - Quaternion magnitude
            - x (float) - Quaternion X axis
            - y (float) - Quaternion Y axis
            - z (float) - Quaternion Z axis
        """
        if self.axis == 6:
            self.w, self.x, self.y, self.z = acc2quat(
                self.ax, self.ay, self.az, nav=self.nav_frame)
        elif self.axis == 9:
            self.w, self.x, self.y, self.z = accmag2quat(
                self.ax, self.ay, self.az, self.mx, self.my, self.mz, nav=self.nav_frame)
        self.quaternion = np.array([[self.w], [self.x], [self.y], [self.z]])
        return self.w, self.x, self.y, self.z

    def noise_chg(self, noise=[]):
        self.noise = noise

    def update_noise_txt(self):
        noise_g = self.get_config_data("noise_g")
        noise_a = self.get_config_data("noise_a")
        noise_m = self.get_config_data("noise_m")
        self.hz = self.get_config_data("hz")
        # print(noise_g,noise_a,noise_m)
        self.noise_chg([noise_g, noise_a, noise_m])

    def get_ahrs_euler(self):
        """
        MPU9250 Euler angle processed by AHRS

        :returns: 
            - roll (float) - x-axis Euler angle in degree
            - pitch (float) - y-axis Euler angle in degree
            - yaw (float) - z-axis Euler angle in degree
        """
        self.w, self.x, self.y, self.z = self.ahrs.run(
            self.acc, self.gyr, self.mag, self.hz, self.noise)

        self.roll, self.pitch, self.yaw = quat2eul(
            self.w, self.x, self.y, self.z, seq=self.rotation_seq)

        if self.degree:
            self.roll, self.pitch, self.yaw = math.degrees(
                self.roll), math.degrees(self.pitch), math.degrees(self.yaw)

            self.roll, self.pitch, self.yaw = round(
                self.roll, 5), round(self.pitch, 5), round(self.yaw, 5)

        self.euler = np.array([[self.roll], [self.pitch], [self.yaw]])
        # self.euler = np.array([[0], [0], [self.yaw]])
        # self.euler = np.array([[0], [self.pitch], [self.yaw]])
        return self.roll, self.pitch, self.yaw

    def get_ahrs_quaternion(self):
        """
        MPU9250 quaternion processed by AHRS

        :returns: 
            - w (float) - Quaternion magnitude
            - x (float) - Quaternion X axis
            - y (float) - Quaternion Y axis
            - z (float) - Quaternion Z axis
        """
        self.w, self.x, self.y, self.z = self.ahrs.run(
            self.acc, self.gyr, self.mag, self.hz, self.noise)

        self.quaternion = np.array([[self.w], [self.x], [self.y], [self.z]])
        return self.w, self.x, self.y, self.z

    def quaternion_multiply(self, quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        self.offset_quaternion = np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                                           x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                                           -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                                           x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)
        return self.offset_quaternion

    def initialization(self):
        """
        Sensor initialization for accelerometer, gyroscope, magnetometer
        """
        self.update_format()
        self.ax, self.ay, self.az = self.get_accel()
        self.gx, self.gy, self.gz = self.get_gyro()
        self.mx, self.my, self.mz = self.get_mag()

        self.roll, self.pitch, self.yaw = self.get_euler()
        self.w, self.x, self.y, self.z = self.get_quaternion()

    def start_thread(self, ahrs=None, sen=None):
        # self.thread_update = Thread(target=self.update_,args=(sen,))
        # self.thread_update.start()
        if ahrs != None:
            self.ahrs = ahrs
            self.ahrs.init_quat(self.w, self.x, self.y, self.z)
            print("filter enable")
        else:
            self.ahrs = None

        self.thread = Thread(target=self.run_calcu, args=(sen, ))
        self.thread.start()

    def update_format(self):
        self.gx_ = self.gyro_raw[0]
        self.gy_ = self.gyro_raw[1]
        self.gz_ = self.gyro_raw[2]

        self.ax_ = self.acc_raw[0]
        self.ay_ = self.acc_raw[1]
        self.az_ = self.acc_raw[2]

        self.mx_ = self.mag_raw[0]
        self.my_ = self.mag_raw[1]
        self.mz_ = self.mag_raw[2]

    def run_calcu(self, sen=None):
        while self.alive:

            if sen != None:
                # print("inside 1")
                self.game_orientation = sen.game_r

                self.acc_raw, self.gyro_raw, self.mag_raw = sen.acc, sen.gyro, sen.mag
                self.update_format()

                # print(self.acc_raw, self.gyro_raw, self.mag_raw)
                self.ax, self.ay, self.az = self.get_accel()
                self.gx, self.gy, self.gz = self.get_gyro()
                self.mx, self.my, self.mz = self.get_mag()

                # self.ax, self.ay, self.az = self.get_accel1_()
                # self.gx, self.gy, self.gz = self.get_gyro1_()
                # self.mx, self.my, self.mz = self.get_mag1_()

                self.acc = np.array([[self.ax], [self.ay], [self.az]])
                self.gyr = np.array([[self.gx], [self.gy], [self.gz]])
                self.mag = np.array([[self.mx], [self.my], [self.mz]])

                # noise_g = self.get_config_data("noise_g")
                # noise_a = self.get_config_data("noise_a")
                # noise_m = self.get_config_data("noise_m")
                # # print(noise_g,noise_a,noise_m)
                # self.noise_chg([noise_g,noise_a,noise_m])

                # self.update_noise_txt()

                if self.ahrs != None:
                    # _, _, _ = self.get_ahrs_euler()
                    self.roll, self.pitch, self.yaw = self.get_ahrs_euler()
                    self.w, self.x, self.y, self.z = self.get_ahrs_quaternion()
                    # print("ahrs roll pitch yaw: ", self.roll, self.pitch, self.yaw)
                else:
                    self.roll, self.pitch, self.yaw = self.get_euler()
                    self.w, self.x, self.y, self.z = self.get_quaternion()

    def out(self):
        print("out_lib")

        self.alive = False

    def get_config_data(self, type):
        # Sub-function to get last port to declare in first run program
        # self.create_config()

        self.ConfigParser.read("config.txt")
        data_config = self.ConfigParser["SETUP"]
        return float(data_config[type])

    def create_config(self):
        if os.path.exists("config.txt") == False:
            print("config file not exists so create one")
            self.ConfigParser["SETUP"] = {
                "noise_g": "0.3",
                "noise_a": "0.5",
                "noise_m": "0.8",
                "hz": "90"
            }
            with open("config.txt", "w") as congfi:
                self.ConfigParser.write(congfi)
                congfi.close()
