#!/usr/bin/python

import numpy as np
import cv2
import matplotlib.pyplot as plt

img = cv2.imread("output/20_low_pass_filtered_image.png")
cv2.imshow('Image', img)
cv2.waitKey(0)
thresh_img = img.copy()

cv2.threshold(img, 20, 255, cv2.THRESH_BINARY, thresh_img)

cv2.imshow('ImageThresh', thresh_img)
cv2.waitKey(0)

np_img = np.array(thresh_img)
coords = np.argwhere(np_img >= 150)
x = coords[:, 0]
y = coords[:, 1]
print(coords[:, 0])
print("**************")
print(coords[:, 1])
xy_coords = zip(*np.asanyarray([x, y]))
print(xy_coords[0])
plt.scatter(x, y)
plt.show()


#def coord_plot(self):
 #   plt.scatter(*self.way_pts_list.T)
  #  plt.show()
    #  print(zip(*self.way_pts_list))
