# Author: Harsh Gupta
# Date: 14th January 2014
# DataSet used: http://cogcomp.cs.illinois.edu/Data/Car/

# TODO:
    # [] after the making dataset and classification
    # [] make a sliding window recognizer based on classification
    # dataset
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm


def hog(img, no_bins=16, orient=False):
    """
    Calculates the histogram of oriented gradient of a grayscale image.

    Parameters
    ----------

    no_bins: no of bins to be used

    orient: If set to True, the function expects a
    2D array of angles of gradient (in radians) corrosponding
    to each pixel
    """

    if not orient:
        gx = cv2.Sobel(img, cv2.CV_32F, 1, 0)
        gy = cv2.Sobel(img, cv2.CV_32F, 0, 1)
        mag, ang = cv2.cartToPolar(gx, gy)
        bins = np.int32(no_bins*ang/(2*np.pi))
    elif orient:
        bins = np.int32(no_bins*img/(2*np.pi))

    hist = cv2.calcHist([bins.astype('float32')], [0], None, [16], [0, 16])
    hist = np.hstack(hist)

    return hist


def hog_vec(img, no_bins=64, orient=False):
    """
    Returns the hog feature vector of a grayscale image.
    It divides the image into four parts and stacks
    the hog of all these parts to get a vector

    Parameters
    ----------

    no_bins: Total no of bins to be used

    orient: If set to True, the function expects a
    2D array of angles of gradient (in radians) corrosponding
    to each pixel
    """

    rows, cols = img.shape
    return np.hstack(hog(img[:rows/2, :cols/2], no_bins=no_bins/4),
                     hog(img[:rows/2, cols/2:], no_bins=no_bins/4),
                     hog(img[rows/2:, cols/2:], no_bins=no_bins/4),
                     hog(img[rows/2:, :cols/2], no_bins=no_bins/4))


class dataset:
    images = []
    data = []
    target = []
    target_names = []
    file_names = []

    def __init__(self):
        pass

    def create_dataset(self, dir_path):
        self.file_names = os.listdir(dir_path)

        self.target = []

        for i, file_name in enumerate(self.file_names):
            if file_name[:3] == "neg":
                self.target.append(0)
            elif file_name[:3] == "pos":
                self.target.append(1)
            else:
                raise NotImplementedError()
            img = cv2.imread(dir_path + file_name, 0)
            self.images.append(img)
            self.data.append(hog_vec(img, no_bins=64))
        self.target = np.array(self.target)


dir_path = "/home/hargup/agv/objectDetection/CarData/TrainImages/"

cars = dataset()
cars.create_dataset(dir_path)

clf = svm.SVC(gamma=0.001, C=100.)
clf.fit(cars.data, cars.target)


def draw_rec(img, x, y, h, w, bw=2):
    if len(img.shape) == 2:
        with_rec = img.copy()
        with_rec[x:x+w, y:y+bw] = 255
        with_rec[x:x+w, y+w:y+w+bw] = 255
        with_rec[x:x+bw, y:y+h] = 255
        with_rec[x+w:x+w+bw, y:y+h] = 255
        return with_rec
    else:
        raise NotImplementedError("draw_rec is not yet implemented"
                                  " for colored images")




# def sliding_window(img, clf):
