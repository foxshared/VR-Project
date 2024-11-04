# TCP way use sensorstream 
from udpserver import UDPServer
import json
import numpy as np
import socket
from threading import Thread
import time


class Sensor_phoneUDP():
    def __init__(self,address,buffer_size = 1024):
        self.alive = True
        self.accr = []
        self.gyror = []
        self.magr = []
        self.game_r = []
        self.orientation=[]

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = address
        self.buffer_size = buffer_size

    def sort_data(self):
        if self.type_ == "android.sensor.accelerometer":
            self.accr = self.values
        elif self.type_ == "android.sensor.gyroscope":
            self.gyror = self.values
        elif self.type_ == "android.sensor.magnetic_field":
            self.magr = self.values
        elif self.type_ == "android.sensor.game_rotation_vector":
            self.game_r = self.values
        

        return self.accr, self.gyror, self.magr,self.game_r
    
    def orientation_(self):
        while self.alive:
            # time.sleep(0.1)
            self.data = self.ws.recv()
            self.values = json.loads(self.data)["values"]

            self.orientation = self.values
        

    def intialize(self):
        self.sock.bind(self.address)
        for i in range(40):
            self.data, self.address = self.sock.recvfrom(self.buffer_size)
            print("count {} = {}".format(i, self.data))
            time.sleep(0.1)

    def loop_sensor(self):
        # self.sock.bind(self.address)
        while self.alive:
            # time.sleep(0.1)
            self.data, self.address = self.sock.recvfrom(self.buffer_size)
            # print(self.data)
            self.type_ = json.loads(self.data)["type"]
            self.values = json.loads(self.data)["values"]

            self.acc, self.gyro, self.mag,self.game_r = self.sort_data()
            # print(self.acc, self.gyro, self.mag,self.game_r)

        self.sock.close()

    def start_thread(self,flag_=True):
        if flag_:
            self.thread = Thread(target=self.loop_sensor)
            self.thread.start()
        
        elif flag_ is False:
            self.thread = Thread(target=self.orientation_)
            self.thread.start()

    def stop_sensor(self):
        
        self.alive = False
