#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String
from skimage import io, color, filters, measure
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

if __name__ == '__main__':

    rospy.init_node('x_ray_position', anonymous=True)
    pubXrayCoords = rospy.Publisher('x_ray_coordinates', Float64MultiArray, queue_size=10)
    xRayCoords = Float64MultiArray()
    xRayCoords.data = [0, 0]

    cap = cv2.VideoCapture('/dev/video2')
    winname = 'Screen Capture'
    cv2.namedWindow(winname)        # Create a named window
    # cv2.moveWindow(winname, 1000,30)

    i = 0
    dt = 40 # ms
    r = rospy.Rate(1000. / dt)
    while not rospy.is_shutdown():
    # while(True):
        ret, image = cap.read()
        # image = cv2.imread('/home/mugurlu/Desktop/Pi3/frame%05d.png' % i)
        imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # thresh = filters.threshold_otsu(imgray)
        # print(imgray)
        # imbin = imgray > thresh
        ret, imbin = cv2.threshold(imgray[440:590,300:700], 145, 255, cv2.THRESH_BINARY)
        imbin = cv2.transpose(imbin)

        kernel = np.ones((24,12),np.uint8)
        imbin = cv2.morphologyEx(imbin.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        label_img = measure.label(imbin, connectivity=imbin.ndim)
        props = measure.regionprops(label_img)

        print('#######')
        for j in range(len(props)):
            if props[j].perimeter < 100 and props[j].perimeter > 60 and props[j].area < 500 and props[j].area > 300:
                imbin[:, props[j].centroid[1].astype(np.uint8)] = abs(imbin[:, props[j].centroid[1].astype(np.uint8)] - 1)
                imbin[props[j].centroid[0].astype(np.uint8), :] = abs(imbin[props[j].centroid[0].astype(np.uint8), :] - 1)
                # imbin[props[j].centroid[0].astype(np.uint8),props[j].centroid[1].astype(np.uint8)] = 0
                # imbin[props[j].centroid[0].astype(np.uint8) + 1,props[j].centroid[1].astype(np.uint8) + 1] = 0
                # imbin[props[j].centroid[0].astype(np.uint8) + 1,props[j].centroid[1].astype(np.uint8) - 1] = 0
                # imbin[props[j].centroid[0].astype(np.uint8) - 1,props[j].centroid[1].astype(np.uint8) + 1] = 0
                # imbin[props[j].centroid[0].astype(np.uint8) - 1,props[j].centroid[1].astype(np.uint8) - 1] = 0
                xRayCoords.data = [props[j].centroid[1], props[j].centroid[0]] # [x,y]
                pubXrayCoords.publish(xRayCoords)
                rospy.loginfo("I published. %s" % xRayCoords.data)
                break
            else:
                print('Centroid\t' + str(props[j].centroid))
                print('Perimeter\t' + str(props[j].perimeter))
                print('Area\t\t' + str(props[j].area))


        # kernel = np.ones((4,4),np.uint8)
        # imbin = cv2.morphologyEx(imbin.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

        cv2.imshow(winname, imbin)

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

        # if cv2.waitKey(100) & 0xFF == ord('q'):
        #     break

        r.sleep()
