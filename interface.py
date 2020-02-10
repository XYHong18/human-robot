# coding: utf-8

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.boxlayout import BoxLayout 
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.image import Image
from kivy.clock import Clock
from kivy.graphics.texture import Texture
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.graphics import *


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



class KinectView(App):

    def build(self):
        
        self.img1=Image()
        self.img1.bind(on_touch_down = self.imageOnPressed)
        self.root = layout = FloatLayout()
        layout.bind(size=self._update_rect, pos=self._update_rect)
        layout.add_widget(self.img1)
        Clock.schedule_interval(self.update_img1, 1/30)

        self.btn1 = Button(text="Close RGB-D Camera", font_size=14, size_hint=(None, None), size=(180, 100), pos=(810, 200))
        self.btn1.bind(on_press=lambda x:self.on_press_btn1())
        layout.add_widget(self.btn1)

        self.btn2 = Button(text="Pause", font_size=14, size_hint=(None, None), size=(180, 100), pos=(1400, 650))
        layout.add_widget(self.btn2)

        self.btn3 = Button(text="Open Gripper", font_size=14, size_hint=(None, None), size=(180, 100), pos=(1400, 420))
        layout.add_widget(self.btn3)


        self.upButton = Button(background_normal="up.png", size_hint=(None, None), pos=(167, 690), border=(0, 0, 0, 0))
        layout.add_widget(self.upButton)

        self.downButton = Button(background_normal="down.png", size_hint=(None, None), pos=(300, 690), border=(0, 0, 0, 0))
        layout.add_widget(self.downButton)

        self.leftButton = Button(background_normal="left.png", size_hint=(None, None), size=(110, 70), pos=(150, 495), border=(0, 0, 0, 0))
        layout.add_widget(self.leftButton)

        self.rightButton = Button(background_normal="right.png", size_hint=(None, None), size=(110, 70), pos=(310, 495), border=(0, 0, 0, 0))
        layout.add_widget(self.rightButton)

        self.backwardButton = Button(background_normal="backward.png", size_hint=(None, None), size=(100, 50), pos=(235, 550), border=(0, 0, 0, 0))
        layout.add_widget(self.backwardButton)

        self.forwardButton = Button(background_normal="forward.png", size_hint=(None, None), size=(135, 91), pos=(216, 428), border=(0, 0, 0, 0))
        layout.add_widget(self.forwardButton)



        with layout.canvas.before:
            Color(1, 1, 1, 1)
            self.rect = Rectangle(size=layout.size, pos=layout.pos)
      
        return layout


    def _update_rect(self, instance, value):
        self.rect.pos = instance.pos
        self.rect.size = instance.size


    def imageOnPressed(self, instance, touch):
        print("The touch is at position: ", self.img1.to_widget(*touch.pos))
        print("Position of Image: ", str(self.img1.to_window(*self.img1.pos)))


    def print_pos(self):
        print("Position: " + str(self.img1.pos))
        print("Position: " + str(self.btn1.pos))


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
        #self.img1.texture = texture1

        with self.img1.canvas:
            Rectangle(pos=(644,426), size=(512, 424), texture=texture1)


        listener.release(frames)
        
         
import os
os.environ['KIVY_GL_BACKEND'] = 'sdl2'

from kivy.config import Config 
  
# 0 being off 1 being on as in true / false 
# you can use 0 or 1 && True or False 
Config.set('graphics', 'resizable', '0') 
  
# fix the width of the window  
Config.set('graphics', 'width', '1800') 
Config.set('graphics', 'height', '900') 



device, listener, fn = open_device()
device.start()
registration = Registration(device.getIrCameraParams(), device.getColorCameraParams())

ir_params = device.getIrCameraParams()
color_params = device.getColorCameraParams()

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

KinectView().run()
cv2.destroyAllWindows()
