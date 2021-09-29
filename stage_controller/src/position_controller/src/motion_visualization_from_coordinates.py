#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String
from skimage import io, color, filters, measure
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

def callback(data):
    global s0_pos, s0_vel, s1_pos, s1_vel, stepper_pos, stepper_vel
    # Structure of data: [theta, w, pos0, vel0, pos1, vel1] in rad, rad/s, mm, mm/s
    s0_pos = data.data[2]
    s0_vel = data.data[3]
    s1_pos = data.data[4]
    s1_vel = data.data[5]

    stepper_pos = data.data[0]
    stepper_vel = data.data[1]



if __name__ == '__main__':
    # define stages and ports as global
    global s0_pos, s0_vel, s1_pos, s1_vel, stepper_pos, stepper_vel

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('coordinates', Float64MultiArray, callback)


    dt = 1000 # ms
    r = rospy.Rate(1000. / dt)
    while not rospy.is_shutdown():

        r.sleep()