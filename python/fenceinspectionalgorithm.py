#!/usr/bin/env python

import cv2
import numpy as np


def load_image():
    img = cv2.imread("input/1.jpg", 0)
    #cv2.namedWindow('fence image', CV_WINDOW_NORMAL)
    resized_img = cv2.resize(img, (1000, 500), interpolation=cv2.INTER_AREA)
    cv2.imshow("fence image", resized_img)
    cv2.waitKey(0)
    return img


class FenceInspection:
    """ Fence inspection class."""
    def __init__(self):
        """ Do something cool """


if __name__ == "__main__":
    inspect = FenceInspection()
    load_image()
