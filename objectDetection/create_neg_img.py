import cv2
import os
import numpy as np

cap = cv2.VideoCapture('/home/hargup/roar.mp4')
dir_path = '/home/hargup/agv/neg_img/'


for i in xrange(50):
    for x in xrange(20):
        ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rows, cols = frame.shape
    for j in xrange(10):
        for k in xrange(10):
            # f = open(dir_path + i)
            cv2.imwrite(dir_path + "neg_" + str(i) + str(j*10 + k) + ".png",
                        frame[rows*j/10:rows*(j+1)/10,
                              cols*k/10: cols*(k+1)/10])
