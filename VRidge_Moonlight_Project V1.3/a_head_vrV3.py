# Use TCP and sensorgram
from construct import *
from construct import Int32ul, Float32l, Int16sl, Int64ul
import zmq
import numpy as np
from vridge_api_pb2 import HeadTrackingRequest, HeadTrackingResponse, ControllerStateRequest, ControllerStateResponse, VRController
import math
import sys
import time

import vridge_api
from sensorfrom_phoneUDP import Sensor_phoneUDP
from collectV3 import collect
from ahrs import madgwick, mahony, ekf
from utils.orientation import eul2quat


def delete_last_line():
    "Use this function to delete the last line in the STDOUT"

    # cursor up one line
    sys.stdout.write('\x1b[1A')

    # delete last line
    sys.stdout.write('\x1b[2K')

def limit_angle(b):
    if 0 <= b <= 180:
        a = b
    else:
        a = b - 360
    
    return a

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

nav_frame = "ENU"  # ENU/NED
axis = 9
hz = 12000
interval = 1/hz
calibration = False
# ahrs = madgwick.Madgwick(axis, 20, nav_frame)
# ahrs = mahony.Mahony(axis, 0.1, 0, nav_frame)
ahrs = ekf.EKF(axis, nav_frame)
coll = collect(nav_frame=nav_frame, axis=axis, hz=hz, calibration=calibration,
                  load_calib=True,mag_sen=[0.8,1,0.8])
coll.initialization()
coll.start_thread(ahrs=ahrs, sen=sen)

last = time.time()

glob_quat = [1, 0, 0, 0]
glob_pyr = [0, 0, 0]
counter = 0
time_ = 0

offset_pyr = [0, 0, 0, 0]

p_n = 0
y_n = 0
r_n = 0


for i in range(1, 500):
    print(i)
    time.sleep(0.001)
    coll.update_noise_txt()
    ini_quat = coll.quaternion
    quat_ini = [ini_quat[0], ini_quat[1], ini_quat[2], ini_quat[3]]

    rpy = coll.euler
    roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
    init_pyr = [pitch, yaw, roll]  # x,z,y
    if i >= 499:
        print("set init")
        break

mode_quaternion = False

try:
    while True:
        # make Angle pitch yaw roll to quaternion
        # create offset
        # send to vridge api

        # next = last + interval
        # a = next - time.time()
        # time.sleep(abs(a))

        coll.update_noise_txt()

        wxyz = coll.quaternion
        w, x, y, z = wxyz[0], wxyz[1], wxyz[2], wxyz[3]

        rpy = coll.euler
        roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
        use_pyr = [pitch, yaw, roll]  # x,z,y

        # Pitch (+up), Yaw (+to left), Roll (+left), X, Y, Z

        if counter == 0 and time_ >= 500:
            offset_pyr = [glob_pyr[0]-init_pyr[0], glob_pyr[1] -
                          init_pyr[1], glob_pyr[2]-init_pyr[2],]

            counter = 1

        time_ += 1

        new_pyr = [-1*(offset_pyr[0]+use_pyr[0]), -1 *
                   (offset_pyr[1]+use_pyr[1]), (offset_pyr[2]+use_pyr[2])]   
        
        if mode_quaternion:
            pack_data = [x, y, z, w]
            vridge_api.send_HeadTrackingRequest(
                # [X, Y, Z, W, X, Y, Z]
                headset, pack_data, 0, 1, 0, tasktype=4)

        elif mode_quaternion == False:
            pyr_data = new_pyr
            p_n = math.radians(pyr_data[0][0])
            y_n = math.radians(pyr_data[1][0])
            r_n = math.radians(pyr_data[2][0])
            pack_data = p_n, y_n, r_n
            vridge_api.send_HeadTrackingRequest(
                headset, pack_data, 0, 1, 0, tasktype=3)
                

        # print(offset_pyr, new_pyr, use_pyr, time_,counter)
        # delete_last_line()

        # last = time.time()

except KeyboardInterrupt:
    sen.stop_sensor()
    # sen1.stop_sensor()
    coll.out()
    print('interrupted!')
