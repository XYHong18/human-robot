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
    global mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
        mouseX, mouseY = x, y
    
        # method 1: use pylibfreenect2 function getPointXYZ
        X, Y, Z = registration.getPointXYZ(undistorted, mouseY, mouseX)
        print("******** Method 1 **********")
        print("mouseX", mouseX, "mouseY", mouseY)
        print("camera coordinates: ", X, -Y, Z)

        # method 2: use intrinsic parameters of depth camera (winner)
        #z = undistorted.asarray(np.float32)[mouseY][mouseX]
        #z_test = bigdepth.asarray(np.float32)[mouseY+1][mouseX]
        #X, Y, Z = depthToPointCloudPos(mouseY, mouseX, z)
        #X_test, Y_test, Z_test = colorToPointCloudPos(mouseY, mouseX, z_test)
        #print("######## Method 2 ###########")
        #print("mouseX", mouseX, "mouseY", mouseY)
        #print("camera coordinates: ", Y, -X, Z)

 

device, listener, fn = open_device()
device.start()
mouseX = 0
mouseY = 0

R = np.array([[ 0.74386487,  0.01682386, -0.66811827],
       [-0.66791034,  0.0541369, -0.74227015],
       [ 0.023682,  0.99839178,  0.05150736]])

t = np.array([0.8073384, 1.15614799, -0.02126742])


# NOTE: must be called after device.start()
registration = Registration(device.getIrCameraParams(),
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

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

# Optinal parameters for registration
# set True if you need
need_bigdepth = True
need_color_depth_map = True

bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
color_depth_map = np.zeros((424, 512),  np.int32).ravel() \
    if need_color_depth_map else None


while True:
    if listener.hasNewFrame():
        frames = listener.waitForNewFrame()

        color = frames["color"]
        ir = frames["ir"]
        depth = frames["depth"]

        registration.apply(color, depth, undistorted, registered,
                       bigdepth=bigdepth,
                       color_depth_map=color_depth_map)

    
        cv2.imshow("registered", registered.asarray(np.uint8))
        cv2.setMouseCallback('registered', draw_circle)
        print(np.max(registered.asarray(np.uint8)))

        listener.release(frames)

        key = cv2.waitKey(delay=1)
        if key == ord('q'):
            break


device.stop()
device.close()

sys.exit(0)

