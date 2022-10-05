#!/usr/bin/env python

"""
auto_exposure_control.py subscribes to a ROS image topic and performs
histogram based automatic exposure control based on the paper
"Automatic Camera Exposure Control", N. Nourani-Vatani, J. Roberts
"""
#import roslib
from cmath import inf
import glob, os
import numpy as np
import cv2

class CameraSimulator:
    def __init__(self, path):
        self.data_dict = self.load_data_dic(path)
        self.cerrent_ev = 0
        self.current_image = self.data_dict[self.cerrent_ev]

    def load_data_dic(self, img_path):
        files = glob.glob(img_path)
        data_dic = {}
        for file in files:
            order = file.split("_")[-1].split(".jpg")[0] # get image order
            order = int(order)
            if order != 1:
                image = cv2.imread(file)
                ev = -1.5 + (order-1)*0.03 # step and initial ev of dataset are -1.5 and 0.03
                ev = self.num2ev(ev)
                data_dic[ev] = image
        return data_dic

    def num2ev(self, num):
        return round(round(num/0.03)*0.03, 2)

    def set(self, new_ev):
        new_ev = self.num2ev(new_ev)
        self.cerrent_ev = new_ev
        self.current_image = self.data_dict[self.cerrent_ev]
    
    def capture(self):
        return self.current_image

class AutoExposure:
    def __init__(self):
        # I term for PI controller
        self.err_i = 0
        self.cerrent_ev = 0

    def get_exposure(self):
        return self.cerrent_ev

    def run(self, cv_image):
        (rows, cols, channels) = cv_image.shape

        if (channels == 3):
            brightness_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)[:,:,2]
        else:
            brightness_image = cv_image

        hist = cv2.calcHist([brightness_image],[0],None,[5],[0,256])
        mean_sample_value = 0
        for i in range(len(hist)):
            mean_sample_value += hist[i]*(i+1)
            
        mean_sample_value /= (rows*cols)

        desired_msv = 2.5
        # Gains
        k_p = 0.05
        k_i = 0.01
        # Maximum integral value
        max_i = 3

        err_p = desired_msv - mean_sample_value
        self.err_i += err_p
        if abs(self.err_i) > max_i:
            self.err_i = np.sign(self.err_i)*max_i
        
        # Don't change exposure if we're close enough. Changing too often slows
        # down the data rate of the camera.
        if abs(err_p) > 0.5:
            add_value = float(k_p*err_p+k_i*self.err_i) #ndarray to float
            self.cerrent_ev += add_value

        return self.get_exposure()

def main():
    path = os.path.join("images","144550", "*")
    cam = CameraSimulator(path)
    ae = AutoExposure()

    initial_ev = 1.3
    cam.set(initial_ev)
    ae.cerrent_ev = initial_ev

    cnt = 0
    ev = initial_ev
    while True:
        print(ev)
        image = cam.capture()
        next_ev = ae.run(image)
        cam.set(next_ev)

        if ev == next_ev:
            cnt += 1
            if cnt == 10:break
        else:cnt = 1

        ev = next_ev

if __name__ == "__main__":
    main()