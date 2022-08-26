#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String
from skimage import io, color, filters, measure
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

# def callback(data):
#     global s0_pos, s1_pos
#     # Structure of data: [theta, w, pos0, vel0, pos1, vel1] in rad, rad/s, mm, mm/s
#     s0_pos = 10000000.*data.data[2]/24.44 + initial_offset0
#     s0_vel = 5000.*data.data[3]
#     s1_pos = 10000000.*data.data[4]/24.44 + initial_offset1
#     s1_vel = 5000.*data.data[5]
#
#     stepper_pos_vel.data = [data.data[0], data.data[1]]

if __name__ == '__main__':

    rospy.init_node('x_ray_position', anonymous=True)
    pubXrayCoords = rospy.Publisher('x_ray_coordinates', Float64MultiArray, queue_size=10)
    # rospy.Subscriber('coordinates', Float64MultiArray, callback)
    xRayCoords = Float64MultiArray()
    xRayCoords.data = [0, 0]

    cap0 = cv2.VideoCapture('/dev/video4')
    cap1 = cv2.VideoCapture('/dev/video2')
    winname0 = 'Screen Capture 0'
    winname1 = 'Screen Capture 1'
    cv2.namedWindow(winname0)        # Create a named window
    cv2.moveWindow(winname0, 1000,30)
    cv2.namedWindow(winname1)        # Create a named window
    cv2.moveWindow(winname1, 10,30)

    i = 0
    dt = 40 # ms
    r = rospy.Rate(1000. / dt)

    # tic = time.clock()
    while not rospy.is_shutdown():
    # while(True):

        # toc = time.clock()
        # print(toc - tic)
        # tic = toc
        ret0, image0 = cap0.read()
        ret1, image1 = cap1.read()
        # imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # # thresh = filters.threshold_otsu(imgray)
        # ret, imbin = cv2.threshold(imgray[440:590,300:700], 25, 255, cv2.THRESH_BINARY)
        # imbin = cv2.flip(cv2.transpose(imbin), 1)

        # kernel = np.ones((10,10),np.uint8)
        # imbin = cv2.morphologyEx(imbin.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        # imbin = cv2.morphologyEx(imbin.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        # label_img = measure.label(imbin, connectivity=imbin.ndim)
        # props = measure.regionprops(label_img)

        # for j in range(len(props)):
            # xRayCoords.data = [props[j].centroid[1], props[j].centroid[0]] # [x,y]
            # pubXrayCoords.publish(xRayCoords)
            # rospy.loginfo("I published. %s" % xRayCoords.data)
            # imbin[:, props[j].centroid[1].astype(np.uint8)] = abs(imbin[:, props[j].centroid[1].astype(np.uint8)] - 1)
            # imbin[props[j].centroid[0].astype(np.uint8), :] = abs(imbin[props[j].centroid[0].astype(np.uint8), :] - 1)
            # print('Centroid\t' + str(props[j].centroid))
            # print('Perimeter\t' + str(props[j].perimeter))
            # print('Area\t\t' + str(props[j].area))


        cv2.imshow(winname0, cv2.rotate(image0, cv2.ROTATE_90_COUNTERCLOCKWISE))
        cv2.imshow(winname1, cv2.rotate(image1, cv2.ROTATE_90_COUNTERCLOCKWISE))

        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

        r.sleep()
