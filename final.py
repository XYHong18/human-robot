# coding: utf-8


import numpy as np
import cv2
import sys
import time
import rospy
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel
from pylibfreenect2 import OpenGLPacketPipeline

import urx
import time
import sys
from threading import Thread, Lock


class register:
    def __init__(self):
        self.registration = None
        self.undistorted = None


def depthToPointCloudPos(x_d, y_d, z, scale=1000):
    x = (x_d - irCameraParams['cx']) * z / irCameraParams['fx']
    y = (y_d - irCameraParams['cy']) * z / irCameraParams['fy']

    return x / scale, y / scale, z / scale


def colorToPointCloudPos(x_d, y_d, z, scale=1000):
    x = (x_d - colorCameraParams['cx']) * z / colorCameraParams['fx']
    y = (y_d - colorCameraParams['cy']) * z / colorCameraParams['fy']

    return x / scale, y / scale, z / scale

        

def open_device():
    # Create a processing pipeline.
    pipeline = OpenGLPacketPipeline()
    print("Packet pipeline:", type(pipeline).__name__)

    # Create a logger.
    logger = createConsoleLogger(LoggerLevel.Debug)
    setGlobalLogger(logger)

    # Discover Kinect device.
    fn = Freenect2()
    num_devices = fn.enumerateDevices()
    if num_devices == 0:
        print("No device connected!")
        sys.exit(1)

    serial = fn.getDeviceSerialNumber(0)

    # Connect to the device.
    device = fn.openDevice(serial, pipeline=pipeline)

    # Register listeners.
    listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)
    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)

    return device, listener, fn


def draw_circle(event, x, y, flags, param):
    global mouseX, mouseY, r_x, r_y, r_z, flag
    if event == cv2.EVENT_LBUTTONDBLCLK:
        mouseX, mouseY = x, y
        print("mouseX", mouseX, "mouseY", mouseY)
        # method 1: use pylibfreenect2 function getPointXYZ
        X, Y, Z = register.registration.getPointXYZ(register.undistorted, mouseY, mouseX)
        print("******** Method 1 **********") 
        print("camera coordinates: X: ", X, "Y: ", -Y, "Z: ", Z)
        camera_coordinates = np.array([X, -Y, Z])
        robot_coordinates = np.dot(R, camera_coordinates.T).T+t
        r_x, r_y, r_z = robot_coordinates[0], robot_coordinates[1], robot_coordinates[2]+0.19
        print("robot coordinates: ", r_x, r_y, r_z)
        flag = True


def robot():
    global flag

    while True:
        if mouseY != -10000 and mouseX != -10000 and flag:
            rob.movel((0.5, 0.4, 0.3, 3, -1, 0), a, v)
            rob.movel((r_x, r_y, r_z, 3, -1, 0), a, v) 
            rob.movel((0.5, 0.4, 0.3, 3, -1, 0), a, v) 
            flag = False      
        else:
            print("The robot is waiting ...")
            


def video():
    # NOTE: must be called after device.start()
    register.registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

    ir_params = device.getIrCameraParams()
    color_params = device.getColorCameraParams()

    # Kinects's intrinsic parameters based on v2 hardware (estimated).
    irCameraParams = {
      "cx":ir_params.cx,
      "cy":ir_params.cy,
      "fx":ir_params.fx,
      "fy":ir_params.fy,
    }


    colorCameraParams = {
      "cx": color_params.cx,
      "cy": color_params.cy,
      "fx": color_params.fx,
      "fy": color_params.fy
    }

    register.undistorted = Frame(512, 424, 4)
    registered = Frame(512, 424, 4)

    # Optinal parameters for registration
    # set True if you need
    need_bigdepth = True
    need_color_depth_map = True

    bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
    color_depth_map = np.zeros((424, 512),  np.int32).ravel() \
        if need_color_depth_map else None
    
    while True:
       frames = listener.waitForNewFrame()

       color = frames["color"]
       ir = frames["ir"]
       depth = frames["depth"]

       register.registration.apply(color, depth, register.undistorted, registered,
                       bigdepth=bigdepth,
                       color_depth_map=color_depth_map)
       
       cv2.imshow("registered", registered.asarray(np.uint8))
       cv2.setMouseCallback('registered', draw_circle)

       listener.release(frames)

       key = cv2.waitKey(delay=1)
       if key == ord('q'):
           break

    device.stop()
    device.close()

    sys.exit(0)



v = 0.05
a = 0.01
rob = urx.Robot("192.168.1.104")

device, listener, fn = open_device()
device.start()
mouseX = -10000
mouseY = -10000
flag = False

R = np.array([[0.80513296, 0.07898627, -0.58781127],
       [-0.59180229, 0.17237556, -0.7874368 ],
       [0.0391276, 0.98185938, 0.1855295 ]])

t = np.array([0.8220288, 1.1116115, -0.01318528])
time.sleep(0.2)


t1 = Thread(target=video)
t1.start()
t2 = Thread(target=robot)
t2.start()





