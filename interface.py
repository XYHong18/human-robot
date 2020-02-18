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

import urx
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

from threading import Thread, Lock

from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel
from pylibfreenect2 import OpenGLPacketPipeline
from threading import Thread



class register:
    def __init__(self):
        self.registration = None
        self.undistorted = None


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
        
        self.img1=Image(on_touch_down = self.on_press_image)
        self.root = layout = FloatLayout()
        layout.bind(size=self._update_rect, pos=self._update_rect)
        layout.add_widget(self.img1)
        Clock.schedule_interval(self.update_img1, 1/50)

        self.cameraButton = Button(text="Close RGB-D Camera", font_size=14, size_hint=(None, None), size=(180, 100), pos=(810, 200))
        self.cameraButton.bind(on_press=lambda x:self.on_press_cameraButton())
        layout.add_widget(self.cameraButton)

        self.pauseButton = Button(text="Stop", font_size=14, size_hint=(None, None), size=(180, 100), pos=(1400, 700))
        self.pauseButton.bind(on_press=lambda x:self.on_press_pauseButton())
        layout.add_widget(self.pauseButton)

        self.homeButton = Button(text="Home Position", font_size=14, size_hint=(None, None), size=(180, 100), pos=(1400, 525))
        self.homeButton.bind(on_press=lambda x:self.on_press_homeButton())
        layout.add_widget(self.homeButton)

        self.gripperButton = Button(text="Open Gripper", font_size=14, size_hint=(None, None), size=(180, 100), pos=(1400, 350))
        self.gripperButton.bind(on_press=lambda x:self.on_press_gripperButton())
        layout.add_widget(self.gripperButton)


        self.upButton = Button(background_normal="up.png", size_hint=(None, None), pos=(167, 690), border=(0, 0, 0, 0))
        self.upButton.bind(on_press=lambda x:self.on_press_upButton())
        layout.add_widget(self.upButton)

        self.downButton = Button(background_normal="down.png", size_hint=(None, None), pos=(300, 690), border=(0, 0, 0, 0))
        self.downButton.bind(on_press=lambda x:self.on_press_downButton())
        layout.add_widget(self.downButton)

        self.leftButton = Button(background_normal="left.png", size_hint=(None, None), size=(110, 70), pos=(150, 495), border=(0, 0, 0, 0))
        self.leftButton.bind(on_press=lambda x:self.on_press_leftButton())
        layout.add_widget(self.leftButton)

        self.rightButton = Button(background_normal="right.png", size_hint=(None, None), size=(110, 70), pos=(310, 495), border=(0, 0, 0, 0))
        self.rightButton.bind(on_press=lambda x:self.on_press_rightButton())
        layout.add_widget(self.rightButton)

        self.backwardButton = Button(background_normal="backward.png", size_hint=(None, None), size=(100, 50), pos=(235, 550), border=(0, 0, 0, 0))
        self.backwardButton.bind(on_press=lambda x:self.on_press_backwardButton())
        layout.add_widget(self.backwardButton)

        self.forwardButton = Button(background_normal="forward.png", size_hint=(None, None), size=(135, 91), pos=(216, 428), border=(0, 0, 0, 0))
        self.forwardButton.bind(on_press=lambda x:self.on_press_forwardButton())
        layout.add_widget(self.forwardButton)


        with layout.canvas.before:
            Color(1, 1, 1, 1)
            self.rect = Rectangle(size=layout.size, pos=layout.pos)
      
        return layout


    def _update_rect(self, instance, value):
        self.rect.pos = instance.pos
        self.rect.size = instance.size


    def on_press_image(self, instance, touch):
        mouseX, mouseY = self.img1.to_widget(*touch.pos)
        print("Mouse Click: ", mouseX, mouseY)

        if IMAGE_X < mouseX < IMAGE_X+512 and IMAGE_Y < mouseY < IMAGE_Y+424:
            Thread(target=self.teleoperation_thread, args=[mouseX, mouseY]).start()
        else:
            print("Mouse click out of range")

    def teleoperation_thread(self, mouseX, mouseY):
        pixel_x, pixel_y = mouseX-IMAGE_X, 424-(mouseY-IMAGE_Y)
        print("$$$$$$$$$", pixel_x, pixel_y)
        X, Y, Z = register.registration.getPointXYZ(register.undistorted, pixel_y, pixel_x)
        camera_coordinates = np.array([X, -Y, Z])
        print("***********", camera_coordinates)
        robot_coordinates = np.dot(R, camera_coordinates.T).T+t
        r_x, r_y, r_z = robot_coordinates[0], robot_coordinates[1], robot_coordinates[2]+0.19

        rob.movel((r_x, r_y, r_z, 3, -1, 0), a, v) 
        
        time.sleep(0.2)


    def on_stop(self):
        #without this, app will not exit even if the window is closed
        device.stop()
        device.close()
        rob.close()
	sys.exit()

    def on_press_cameraButton(self):
        Thread(target=self.camera_thread).start()


    def camera_thread(self):

        if self.cameraButton.text == "Open RGB-D Camera":
            self.cameraButton.text = "Close RGB-D Camera"
            Clock.schedule_interval(self.update_img1, 1/50)
        elif self.cameraButton.text == "Close RGB-D Camera":
            self.cameraButton.text = "Open RGB-D Camera"
            Clock.unschedule(self.update_img1)


    def on_press_pauseButton(self):
        Thread(target=self.pause_thread).start()

    def pause_thread(self):
        rob.stopl()

    def on_press_homeButton(self):
        Thread(target=self.home_thread).start()

    def home_thread(self):
        rob.movel((0.5, 0.4, 0.3, 3, -1, 0), a, v) 
        time.sleep(0.2)


    def on_press_gripperButton(self):
        Thread(target=self.gripper_thread).start()

    def gripper_thread(self):
        if self.gripperButton.text == "Open Gripper":
            self.gripperButton.text = "Close Gripper"
            robotiqgrip = Robotiq_Two_Finger_Gripper(rob)
            robotiqgrip.open_gripper()
        elif self.gripperButton.text == "Close Gripper":
            self.gripperButton.text = "Open Gripper"
            robotiqgrip = Robotiq_Two_Finger_Gripper(rob)
            robotiqgrip.close_gripper()

    def on_press_upButton(self):
        Thread(target=self.move_thread, args=["up"]).start()

    def on_press_downButton(self):
        Thread(target=self.move_thread, args=["down"]).start()

    def on_press_leftButton(self):
        Thread(target=self.move_thread, args=["left"]).start()

    def on_press_rightButton(self):
        Thread(target=self.move_thread, args=["right"]).start()

    def on_press_backwardButton(self):
        Thread(target=self.move_thread, args=["backward"]).start()

    def on_press_forwardButton(self):
        Thread(target=self.move_thread, args=["forward"]).start()

    def move_thread(self, direction):
        rob_pos = rob.getl()
        
        rob_x, rob_y, rob_z = rob_pos[0], rob_pos[1], rob_pos[2]
        rob_pos = np.array([rob_x, rob_y, rob_z])
        camera_pos = np.dot(inversed_R, rob_pos.T).T + inversed_t
     
        cam_x, cam_y, cam_z = camera_pos[0], camera_pos[1], camera_pos[2]

        if direction == "up":
            camera_pos = np.array([cam_x, cam_y+MOVE_UNIT, cam_z])
            rob_pos = np.dot(R, camera_pos.T).T + t
            r_x, r_y, r_z = rob_pos[0], rob_pos[1], rob_pos[2]
            
            
        if direction == "down":
            camera_pos = np.array([cam_x, cam_y-MOVE_UNIT, cam_z])
            rob_pos = np.dot(R, camera_pos.T).T + t
            r_x, r_y, r_z = rob_pos[0], rob_pos[1], rob_pos[2]

        if direction == "left":
            camera_pos = np.array([cam_x-MOVE_UNIT, cam_y, cam_z])
            rob_pos = np.dot(R, camera_pos.T).T + t
            r_x, r_y, r_z = rob_pos[0], rob_pos[1], rob_pos[2]

        if direction == "right":
            camera_pos = np.array([cam_x+MOVE_UNIT, cam_y, cam_z])
            rob_pos = np.dot(R, camera_pos.T).T + t
            r_x, r_y, r_z = rob_pos[0], rob_pos[1], rob_pos[2]

        if direction == "forward":
            camera_pos = np.array([cam_x, cam_y, cam_z-MOVE_UNIT])
            rob_pos = np.dot(R, camera_pos.T).T + t
            r_x, r_y, r_z = rob_pos[0], rob_pos[1], rob_pos[2]

        if direction == "backward":
            camera_pos = np.array([cam_x, cam_y, cam_z+MOVE_UNIT])
            rob_pos = np.dot(R, camera_pos.T).T + t
            r_x, r_y, r_z = rob_pos[0], rob_pos[1], rob_pos[2]

        rob.movel((r_x, r_y, r_z, 3, -1, 0), a, v) 

    
    def update_img1(self, dt):

        frames = listener.waitForNewFrame()

        color = frames["color"]
        ir = frames["ir"]
        depth = frames["depth"]

        register.registration.apply(color, depth, register.undistorted, registered,
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
            Rectangle(pos=(IMAGE_X,IMAGE_Y), size=(512, 424), texture=texture1)

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

v = 0.05
a = 0.01
rob = urx.Robot("192.168.1.104")

time.sleep(0.2)

device, listener, fn = open_device()
device.start()
register.registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())


ir_params = device.getIrCameraParams()
color_params = device.getColorCameraParams()

register.undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

R = np.array([[0.80513296, 0.07898627, -0.58781127],
              [-0.59180229, 0.17237556, -0.7874368 ],
              [0.0391276, 0.98185938, 0.1855295 ]])
t = np.array([0.8220288, 1.1116115, -0.01318528])

inversed_R = np.array([[0.78760051, -0.61546286, 0.02984794],
                       [0.10371184, 0.18015614, 0.97815521],
                       [-0.6073955, -0.76729996, 0.20572185]])
inversed_t = np.array([0.01993442, -0.25646342, 1.35431799])

MOVE_UNIT = 0.01
IMAGE_X = 644
IMAGE_Y = 426

KinectView().run()
cv2.destroyAllWindows()
