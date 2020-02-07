# coding: utf-8

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.clock import Clock
from kivy.graphics.texture import Texture
from kivy.uix.button import Button
import numpy as np
import cv2
import sys
import time
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel
from pylibfreenect2 import OpenGLPacketPipeline
from threading import Thread


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


class CamApp(App):

    def build(self):
        self.img1=Image()
        layout = BoxLayout()
        layout.add_widget(self.img1)
        self.btn1 = Button(text="Open RGB-D Camera", pos=(300,350), font_size=12, size_hint = (.18, .12))
        self.btn1.bind(on_press=lambda x:self.on_press_btn1())
        layout.add_widget(self.btn1)
        
        return layout


    def on_stop(self):
        #without this, app will not exit even if the window is closed
        device.stop()
        device.close()


    def on_press_btn1(self):
        if self.btn1.text == "Open RGB-D Camera":
            self.btn1.text = "Close RGB-D Camera"
            Clock.schedule_interval(self.update_img1, 1/30)
        elif self.btn1.text == "Close RGB-D Camera":
            self.btn1.text = "Open RGB-D Camera"
            Clock.unschedule(self.update_img1)
    
    def update_img1(self, dt):
        
        if listener.hasNewFrame():
            
            frames = listener.waitForNewFrame()

            color = frames["color"]
            ir = frames["ir"]
            depth = frames["depth"]

            registration.apply(color, depth, undistorted, registered,
                       bigdepth=None,
                       color_depth_map=None)

            colors = registered.asarray(np.uint8)
            colors = cv2.cvtColor(colors, cv2.COLOR_BGRA2RGB)

            #convert it to texture
            texture1 = Texture.create(size=(colors.shape[1], colors.shape[0]), colorfmt='rgb')
            texture1.flip_vertical()
            #texture1.flip_horizontal()
            texture1.blit_buffer(colors.tostring(), colorfmt='rgb', bufferfmt='ubyte')
            # display image from the texture
            self.img1.texture = texture1
            listener.release(frames)
         
       

device, listener, fn = open_device()
device.start()
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

ir_params = device.getIrCameraParams()
color_params = device.getColorCameraParams()

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

CamApp().run()
cv2.destroyAllWindows()
