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
import pickle


def hog(img, no_bins=16, orient=False):
    """
    Calculates the normalised histogram of oriented gradient of a grayscale image.

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

    return (hist*1000)/(img.shape[0]*img.shape[1])


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
    return np.hstack([hog(img[:rows/2, :cols/2], no_bins=no_bins/4),
                     hog(img[:rows/2, cols/2:], no_bins=no_bins/4),
                     hog(img[rows/2:, cols/2:], no_bins=no_bins/4),
                     hog(img[rows/2:, :cols/2], no_bins=no_bins/4)])


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

            #XXX carefull using hog now
            self.data.append(hog(img, no_bins=16))
        self.target = np.array(self.target)


dir_path = "/home/hargup/agv/objectDetection/CarData/TrainImages/"

clf_path = "/home/hargup/agv/objectDetection/classifiers/car_svm_clf.pkl"

try:
    clf = pickle.load(open(clf_path, 'rb'))
except IOError:
    cars = dataset()
    cars.create_dataset(dir_path)

    clf = svm.SVC(gamma=0.001, C=100.)
    clf.fit(cars.data, cars.target)
    pickle.dump(clf, open(clf_path, 'wb'))


def draw_rec(img, x, y, h, w, bw=2):
    if len(img.shape) == 2:
        # img = img.copy()
        img[x:x+w, y:y+bw] = 255
        img[x:x+w, y+w:y+w+bw] = 255
        img[x:x+bw, y:y+h] = 255
        img[x+w:x+w+bw, y:y+h] = 255
        return img
    else:
        raise NotImplementedError("draw_rec is not yet implemented"
                                  " for colored images")


def sliding_window(img, clf, min_win_size=40):
    rows, cols = img.shape

    # # precalculating the angles of gradient
    # # to improve performance
    # gx = cv2.Sobel(img, cv2.CV_32F, 1, 0)
    # gy = cv2.Sobel(img, cv2.CV_32F, 0, 1)
    # mag, ang = cv2.cartToPolar(gx, gy)

    for w in range(min_win_size, rows):
        for h in range(min_win_size, cols):
            for x in range(0, rows - w):
                for y in range(0, cols - w):
                    data = hog(img[x:x+w, y:y+h], no_bins=16)
                    if clf.predict(data)[0] == 1:
                        draw_rec(img, x, y, w, h)
                        return
