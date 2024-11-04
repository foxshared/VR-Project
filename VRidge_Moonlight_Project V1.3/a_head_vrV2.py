# Use TCP and sensorgram
from construct import *
from construct import Int32ul, Float32l, Int16sl, Int64ul
import zmq
import numpy as np
from vridge_api_pb2 import HeadTrackingRequest, HeadTrackingResponse, ControllerStateRequest, ControllerStateResponse, VRController
from math import *
import sys
import time

import vridge_api
import data2rotpos
from sensorfrom_phoneUDP import Sensor_phoneUDP
from collectV2 import collect
from ahrs import madgwick, mahony, ekf
from utils.orientation import eul2quat


def delete_last_line():
    "Use this function to delete the last line in the STDOUT"

    # cursor up one line
    sys.stdout.write('\x1b[1A')

    # delete last line
    sys.stdout.write('\x1b[2K')


# ================================================================
# Setup vridge connection
ctx, master = vridge_api.connect_control_channel()
headset_addr = vridge_api.check_headset_port(master)
controller_addr = vridge_api.check_controller_port(master)
headset = vridge_api.connect_headset(ctx, headset_addr)
controller = vridge_api.connect_controller(ctx, controller_addr)


# ================================================================
address = ("0.0.0.0", 8080)
# connect phone server from sensorfrom_phone library
# ================================================================


sen = Sensor_phoneUDP(address=address)
sen.intialize()
sen.start_thread(True)

# sen1 = Sensor_phone(url=url1)
# sen1.intialize()
# sen1.start_thread(False)

nav_frame = "NED"  # ENU/NED
axis = 9
hz = 10000
interval = 1/hz
calibration = False
ahrs = madgwick.Madgwick(axis, 20, nav_frame)
# ahrs = mahony.Mahony(axis, 0.1, 0, nav_frame)
# ahrs = ekf.EKF(axis, [0.3**2, 0.5**2, 0.8**2], nav_frame)
coll = collect(nav_frame=nav_frame, axis=9, hz=hz,
               calibration=calibration, load_calib=True)
coll.initialization()
coll.start_thread(ahrs=ahrs, sen=sen)

last = time.time()

glob_quat = [1, 0, 0, 0]
glob_pyr = [0, 0, 0]
counter = 0


for i in range(1, 101):
    print(i)
    if i >= 100:
        ini_quat = coll.quaternion
        quat_ini = [ini_quat[0], ini_quat[1], -ini_quat[2], -ini_quat[3]]

        rpy = coll.euler
        pitch, yaw, roll = rpy[1], rpy[2], rpy[0]
        init_pyr = [radians(roll), -radians(yaw), radians(pitch)]  # x,z,y

        print("set init")

mode_quaternion = False

try:
    while True:
        next = last + interval
        a = next - time.time()
        # print(a)
        time.sleep(abs(a))

        wxyz = coll.quaternion
        current_quat = wxyz[0], wxyz[1], -wxyz[2], -wxyz[3]

        rpy = coll.euler
        pitch, yaw, roll = rpy[1], rpy[2], rpy[0]
        use_pyr = [radians(roll), -radians(yaw), radians(pitch)]  # x,z,y

        if counter == 0:
            quat_first = [quat_ini[0], -
                          quat_ini[1], -quat_ini[2], -quat_ini[3]]
            offset = coll.quaternion_multiply(glob_quat, quat_first)

            offset_pyr = [glob_pyr[0]-init_pyr[0], glob_pyr[1] -
                          init_pyr[1], glob_pyr[2]-init_pyr[2],]

            counter = 1

        quat_calib = coll.quaternion_multiply(offset, current_quat)

        w, x, y, z = quat_calib[0], quat_calib[1], quat_calib[2], quat_calib[3]

        calib_pyr = [use_pyr[0]+offset_pyr[0], use_pyr[1] +
                     offset_pyr[1], use_pyr[2]+offset_pyr[2],]
        w_, x_, y_, z_ = eul2quat(
            calib_pyr[0], calib_pyr[2], calib_pyr[1], seq="zyx")

        # px, py, pz, pw = coll.game_orientation
        # print("w{}, x{}, y{}, z{} ||||| w {},x {},y {},z {} :::::w{}, x{}, y{}, z{} ".format(
        #     w, x, y, z, w_, x_, y_, z_, pw, px, py, pz))

        if mode_quaternion:
            pack_data = [x, y, z, w]
            vridge_api.send_HeadTrackingRequest(
                # [X, Y, Z, W, X, Y, Z]
                headset, pack_data, 0, 1, 0, tasktype=4)

        elif mode_quaternion == False:
            pack_data = calib_pyr
            # Pitch (+up), Yaw (+to left), Roll (+left), X, Y, Z
            vridge_api.send_HeadTrackingRequest(
                headset, pack_data, 0, 1, 0, tasktype=3)

        # delete_last_line()

        last = time.time()

except KeyboardInterrupt:
    sen.stop_sensor()
    # sen1.stop_sensor()
    coll.out()
    print('interrupted!')
