#!/usr/bin/env python3
import rospy
import numpy as np
import scipy.linalg
from std_msgs.msg import Float64MultiArray, String


if __name__ == '__main__':
    global pubCoords, coords, dt, publish
    dt = 1. # s 
    publish = True

    rospy.init_node('stage_path_publisher', anonymous=True)
    pubCoords = rospy.Publisher('coordinates', Float64MultiArray, queue_size=10)
    rospy.sleep(2.)

    coords = Float64MultiArray()
    coords.data = [0,0,0,0,0,0]

    v_x = 10 # mm/s given constant
    v_y = 10
    w = 0

    coords.data[3] = abs(v_x)
    coords.data[5] = abs(v_y)

    interval = 2.5 # mm
    x_init = -2.5
    y_init = 0
    x_end = 20
    y_end = -80
    y_offset = -1.5
    x_offset = (39.5-20.)/2.
    x = np.linspace(x_init + x_offset, x_end + x_offset, int(abs(x_end - x_init)/interval +1))
    y = np.linspace(y_init + y_offset, y_end + y_offset, int(abs(y_end - y_init)/interval +1))

    x_grids = np.array([])
    path = np.meshgrid(x, y)
    for i in range(y.size):
        if i%2:
            x_grids = np.append(x_grids, np.flip(x))
        else:
            x_grids = np.append(x_grids, x)
    # x_grids = np.ndarray.flatten(path[0])
    y_grids = np.ndarray.flatten(path[1])
    i = 0

    rospy.loginfo("I will scan %s points."%(x.size * y.size))
    rospy.loginfo("Estimated time is %s minutes."%(x.size * y.size * dt / 60.0))

    r = rospy.Rate(1./dt)
    while i < x.size * y.size:
        coords.data[2] = x_grids[i]
        coords.data[4] = y_grids[i]
        rospy.loginfo("I published. %s"%coords.data)

        pubCoords.publish(coords)
            # publish = False

        i = i + 1

        r.sleep()


    coords.data[2] = 0
    coords.data[4] = 0
    rospy.loginfo("I published. %s"%coords.data)

    pubCoords.publish(coords)
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()