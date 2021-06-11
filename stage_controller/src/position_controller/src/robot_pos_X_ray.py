#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String
from skimage import io, color, filters, measure
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
i = 1000
tic = time.clock()
while(i < 1100):
    # print('/home/mugurlu/Desktop/Pi3/frame%05d.png' % i)
    image = cv2.imread('/home/mugurlu/Desktop/Pi3/frame%05d.png' % i)
    imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # image = io.imread('/home/mugurlu/Desktop/Pi3/frame01000.png')
    # imgray = color.rgb2gray(image[361:526,401:456])

    # thresh = filters.threshold_otsu(imgray)
    # print(imgray)
    # imbin = imgray > thresh
    ret, imbin = cv2.threshold(imgray[361:526,401:456], 145, 255, cv2.THRESH_BINARY)
    # print(thresh)

    kernel = np.ones((7,4),np.uint8)
    imbin = cv2.morphologyEx(imbin.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    label_img = measure.label(imbin, connectivity=imbin.ndim)
    props = measure.regionprops(label_img)
    print('#######')
    for j in range(len(props)):
        # print(str(props[j].centroid))
        print('Perimeter\t' + str(props[j].perimeter))
        print('Area\t\t' + str(props[j].area))
    # kernel = np.ones((4,4),np.uint8)
    # imbin = cv2.morphologyEx(imbin.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

    cv2.imshow('', imbin)

    # fig, axes = plt.subplots(ncols=3, figsize=(8, 6), )
    # ax = axes.ravel()
    # ax[0] = plt.subplot(1, 3, 1)
    # ax[1] = plt.subplot(1, 3, 2)
    # ax[2] = plt.subplot(1, 3, 3, sharex=ax[0], sharey=ax[0])
    #
    # ax[0].imshow(imgray, cmap=plt.cm.gray)
    # ax[0].set_title('Original')
    # ax[0].axis('off')
    #
    # ax[1].hist(imgray.ravel(), bins=256)
    # ax[1].set_title('Histogram')
    # ax[1].axvline(thresh, color='r')
    #
    # ax[2].imshow(imbin, cmap=plt.cm.gray)
    # ax[2].set_title('Thresholded')
    # ax[2].axis('off')

    # io.imshow(imgray)
    # io.imshow(imbin)
    # plt.show()

    i = i + 1

    if cv2.waitKey(3) & 0xFF == ord('q'):
        break

toc = time.clock()
print(toc - tic)