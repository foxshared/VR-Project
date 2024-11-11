from collectV3 import collect
import yaml
import numpy as np
from ahrs import madgwick, mahony, ekf
import time
from sensorfrom_phoneUDP import Sensor_phoneUDP

if __name__ == '__main__':
    # Code to calibrated 
    # TCP Version
    # url = 'ws://192.168.100.133:8090/sensors/connect?types=["android.sensor.accelerometer","android.sensor.gyroscope","android.sensor.magnetic_field"]'
    # sen = Sensor_phone(url=url)
    # sen.intialize()
    # sen.start_thread()

    address=("0.0.0.0", 8080)
    sen = Sensor_phoneUDP(address=address)
    sen.intialize()
    sen.start_thread(True)

    nav_frame = "ENU"  # ENU/NED
    hz = 3000
    interval = 1/hz
    acc_time = 1
    gyro_time = 1
    mag_time = 20
    calibration = True
    con = collect(nav_frame=nav_frame, calibration=calibration, hz=hz,load_calib=False)
    con.initialization()
    con.start_thread(ahrs=None, sen=sen)

    gyro_scale, gyro_bias, gyro_misalignment = con.gyro_calibration(gyro_time)
    accel_scale, accel_bias, accel_misalignment = con.accel_calibration(acc_time)
    mag_scale, mag_bias, mag_misalignment, mag_strength = con.mag_calibration(mag_time)
    print("done")
    if calibration:
        print("calibration")
        bias = np.vstack((np.vstack((gyro_bias, accel_bias)), mag_bias))
        scale = np.vstack((np.vstack((gyro_scale, accel_scale)), mag_scale))
        misalignment = np.vstack(
            (np.vstack((gyro_misalignment, accel_misalignment)), mag_misalignment))

        # Refresh new config to yaml file
        config = yaml.load(open("config.yaml", "r"), Loader=yaml.FullLoader)
        bias_parameter = ["gx_bias", "gy_bias", "gz_bias", "ax_bias",
                          "ay_bias", "az_bias", "mx_bias", "my_bias", "mz_bias"]
        scale_parameter = ["gx_scale", "gy_scale", "gz_scale", "ax_scale",
                           "ay_scale", "az_scale", "mx_scale", "my_scale", "mz_scale"]

        bias_scale = {}
        for i, element in enumerate(bias_parameter):
            bias_scale[element] = float(bias[i][0])
        for i, element in enumerate(scale_parameter):
            bias_scale[element] = float(scale[i][0])
        config[nav_frame] = bias_scale
        with open("config.yaml", 'w') as file:
            file.write(yaml.dump(config))
        print("accel bias: ")
        print(accel_bias)
        print("gyro bias: ")
        print(gyro_bias)

        print("mag bias: ")
        print(mag_bias)
        print("accel scale: ")
        print(accel_scale)
        print("gyro scale: ")
        print(gyro_scale)
        print("mag scale: ")
        print(mag_scale)
    sen.stop_sensor()
    con.out()
