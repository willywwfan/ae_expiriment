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
import sys

class AutoExposure:
    def __init__(self, path):
        # I term for PI controller
        self.err_i = 0
        self.data_dict = self.load_data_dic(path)
        self.cerrent_ev = -1.47
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

    def get_exposure(self):
        return self.cerrent_ev
    
    def get_img(self):
        return self.current_image
    
    def num2ev(self,num):
        return round(round(num/0.03)*0.03, 2)

    def set_situation(self, new_ev):
        new_ev = self.num2ev(new_ev)
        self.cerrent_ev = new_ev
        print(self.cerrent_ev)
        self.current_image = self.data_dict[self.cerrent_ev]

    def image_callback(self):
        cv_image = self.get_img()
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
            add_value = float(k_p*err_p+k_i*self.err_i)
            self.set_situation(self.cerrent_ev + add_value)

def main():
    path = os.path.join("images","144550", "*")
    ae = AutoExposure(path)
    cnt = 0
    while True:
        print(ae.get_exposure())
        ev = ae.get_exposure()
        ae.image_callback()
        
        if ev == ae.get_exposure():
            cnt += 1
            if cnt == 10:
                break
        else:
            cnt = 1
if __name__ == "__main__":
    main()