#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tkinter as tk
import threading

class ColorPicker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.cv_image = None

        self.root = tk.Tk()
        self.root.title("HSV Color Picker")

        self.lower_h = tk.IntVar(value=0)
        self.lower_s = tk.IntVar(value=0)
        self.lower_v = tk.IntVar(value=0)
        self.upper_h = tk.IntVar(value=179)
        self.upper_s = tk.IntVar(value=255)
        self.upper_v = tk.IntVar(value=255)

        self.create_widgets()

    def create_widgets(self):
        self.frame = tk.Frame(self.root)
        self.frame.pack()

        self.lower_h_scale = tk.Scale(self.frame, from_=0, to=179, label="Lower H", variable=self.lower_h, orient=tk.HORIZONTAL)
        self.lower_h_scale.pack()
        self.lower_s_scale = tk.Scale(self.frame, from_=0, to=255, label="Lower S", variable=self.lower_s, orient=tk.HORIZONTAL)
        self.lower_s_scale.pack()
        self.lower_v_scale = tk.Scale(self.frame, from_=0, to=255, label="Lower V", variable=self.lower_v, orient=tk.HORIZONTAL)
        self.lower_v_scale.pack()

        self.upper_h_scale = tk.Scale(self.frame, from_=0, to=179, label="Upper H", variable=self.upper_h, orient=tk.HORIZONTAL)
        self.upper_h_scale.pack()
        self.upper_s_scale = tk.Scale(self.frame, from_=0, to=255, label="Upper S", variable=self.upper_s, orient=tk.HORIZONTAL)
        self.upper_s_scale.pack()
        self.upper_v_scale = tk.Scale(self.frame, from_=0, to=255, label="Upper V", variable=self.upper_v, orient=tk.HORIZONTAL)
        self.upper_v_scale.pack()

        self.quit_button = tk.Button(self.frame, text="Quit", command=self.quit)
        self.quit_button.pack()

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def quit(self):
        self.root.quit()

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
                
                lower_color = np.array([self.lower_h.get(), self.lower_s.get(), self.lower_v.get()])
                upper_color = np.array([self.upper_h.get(), self.upper_s.get(), self.upper_v.get()])

                mask = cv2.inRange(hsv_image, lower_color, upper_color)
                result = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)

                cv2.imshow("Result", result)
                cv2.waitKey(1)

            self.root.update_idletasks()
            self.root.update()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('color_picker_node', anonymous=True)
    color_picker = ColorPicker()
    color_picker.run() 