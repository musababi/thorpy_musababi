#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String
from skimage import io, color, filters, measure
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

def callback(data):
    global x0, y0, x1, y1, x2, y2, x3, y3
    # Structure of data: [theta, w, pos0, vel0, pos1, vel1] in rad, rad/s, mm, mm/s
    s0_pos = data.data[2]
    s0_vel = data.data[3]
    s1_pos = data.data[4]
    s1_vel = data.data[5]
    stepper_pos = data.data[0]
    stepper_vel = data.data[1]

    x2 = int(75 - s0_pos)
    y2 = int(200 - s1_pos)
    x0 = int(x2 - 20 * np.cos(stepper_pos))
    y0 = int(y2 + 20 * np.sin(stepper_pos))
    x1 = int(x2 + 20 * np.cos(stepper_pos))
    y1 = int(y2 - 20 * np.sin(stepper_pos))
    x3 = int(x2 + 40 * np.sin(stepper_pos))
    y3 = int(y2 + 40 * np.cos(stepper_pos))


if __name__ == '__main__':
    # define stages and ports as global
    global x0, y0, x1, y1, x2, y2, x3, y3

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('coordinates', Float64MultiArray, callback)

    height = 400
    width = 150
    image = np.zeros((height, width, 3), np.uint8)

    winname = 'Screen Capture'
    cv2.namedWindow(winname)        # Create a named window
    cv2.moveWindow(winname, 30, 30)

    x2 = 75
    y2 = 200
    x0 = x2 - 20
    y0 = y2
    x1 = x2 + 20
    y1 = y2
    x3 = x2
    y3 = y2 + 40

    cv2.line(image, (x0, y0), (x1, y1), (127, 127, 0), 2)
    cv2.line(image, (x2, y2), (x3, y3), (127, 127, 0), 2)

    cv2.imshow(winname, image)
    cv2.waitKey(1000)
    print('---------------------------')


    dt = 50 # ms
    r = rospy.Rate(1000. / dt)
    while not rospy.is_shutdown():
        image = np.zeros((height, width, 3), np.uint8)
        cv2.line(image, (x0, y0), (x1, y1), (127, 127, 0), 2)
        cv2.line(image, (x2, y2), (x3, y3), (127, 127, 0), 2)

        cv2.imshow(winname, image)
        cv2.waitKey(10)
        print('---------------------------')

        r.sleep()