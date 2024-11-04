import zmq
from construct import *
from construct import Int32ul, Float32l, Int16sl, Int64ul
from vridge_api_pb2 import HeadTrackingRequest, HeadTrackingResponse, ControllerStateRequest, ControllerStateResponse, VRController
# Vridge API setup


def connect_control_channel():
    # default address connect vridge api 'tcp://localhost:38219'
    m_addr = 'tcp://localhost:38219'
    # -------------------------------
    # connect to the control channel
    print("Connecting to", m_addr)
    ctx = zmq.Context()

    master = ctx.socket(zmq.REQ)
    master.connect(m_addr)

    master.send_json({"ProtocolVersion": 1, "Code": 2})
    answer = master.recv_json()

    print(answer)
    return ctx, master


def check_headset_port(master):
    master.send_json({"RequestedEndpointName": "HeadTracking",
                      "ProtocolVersion": 3, "Code": 1})
    answer = master.recv_json()
    print(answer)

    if answer['ProtocolVersion'] != 3 or answer['Code'] != 0:
        print("HeadTracking: incompatible protocol, or not available")
        exit(0)

    headset_addr = answer['Port']
    headset_addr = "tcp://localhost:{}".format(headset_addr)
    # headset_addr = answer['EndpointAddress']

    print("Found Head")
    print("Headset", headset_addr)
    return headset_addr


def check_controller_port(master):
    master.send_json({"RequestedEndpointName": "Controller",
                      "ProtocolVersion": 3, "Code": 1})
    answer = master.recv_json()
    print(answer)

    if answer['ProtocolVersion'] != 3 or answer['Code'] != 0:
        print("Controller: incompatible protocol, or not available")
        exit(0)

    controller_addr = answer['Port']
    controller_addr = "tcp://localhost:{}".format(controller_addr)
    # controller_addr = answer['EndpointAddress']

    print("Found Controller")
    print("Controller", controller_addr)
    return controller_addr


def connect_headset(ctx, headset_addr):
    # -------------------------------
    # Connect to headset
    headset = ctx.socket(zmq.REQ)
    headset.connect(headset_addr)

    print("connect headset")

    return headset


def connect_controller(ctx, controller_addr):
    # -------------------------------
    # Connect to Controller
    controller = ctx.socket(zmq.REQ)
    controller.connect(controller_addr)

    print("connect Controller")

    return controller


def send_HeadTrackingRequest(headset, pack_data, Xp, Yp, Zp, tasktype):
    # Protobuf serialized vridge api
    # Headtrack send read data
    send_track = HeadTrackingRequest()
    send_track.Version = 3
    # Send [X, Y, Z, W, X, Y, Z] (quaternion, then position) mode 4
    send_track.TaskType = tasktype
    # Change total data need to send

    # data = Struct("data"/Padded(64, Array(7, Float32l)))
    # data struct [X, Y, Z, W, X, Y, Z] (quaternion, then position)
    # data1 = dict(data=[0, 0.1, 0, 0, 0, 1, 0])

    if len(pack_data) == 4 and tasktype == 4:
        data = Struct("data"/Padded(64, Array(7, Float32l)))
        Xq, Yq, Zq, Wq = pack_data[0], pack_data[1], pack_data[2], pack_data[3]
        data1 = dict(data=[Xq, Yq, Zq, Wq, Xp, Yp, Zp])
        # print(len(pack_data),data1)

    elif len(pack_data) == 3 and tasktype == 3:
        data = Struct("data"/Padded(64, Array(6, Float32l)))
        pitch, yaw, roll = pack_data[0], pack_data[1], pack_data[2]
        data1 = dict(data=[pitch, yaw, roll, Xp, Yp, Zp])
        # print(len(pack_data),data1)

    # build data and read
    data_r = data.build(data1)
    send_track.Data = data_r
    serialized_track = send_track.SerializeToString()
    # print(serialized_track)
    # read_back = HeadTrackingRequest()
    # read_back.ParseFromString(serialized_track)
    # print(read_back)

    # ================================================================
    # Send byte data to vridge api
    headset.send(serialized_track)

    # Receive and read data from vridge api
    answer1 = headset.recv()
    # print(answer1)
    # read_answer1 = HeadTrackingResponse()
    # read_answer1.ParseFromString(answer1)
    # print(read_answer1)
    # End
    # ================================================================
