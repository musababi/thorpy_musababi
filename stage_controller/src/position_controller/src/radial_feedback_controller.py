#!/usr/bin/env python3
from thorpy.comm.discovery import discover_stages
import rospy
from std_msgs.msg import Float64MultiArray, String

def getInitialCoordinates(data):
    global coords, publish
    coords.data[0] = data.data[0]
    coords.data[2] = data.data[1]
    coords.data[4] = data.data[2]
    publish = True
    # subCurrentCoords.unregister()

def callback_coordinates(data):


def callback_x_ray_coords(data):


if __name__ == '__main__':

    publish = False

    rospy.init_node('radial_feedback_controller', anonymous=True)


    rospy.Subscriber('coordinates', Float64MultiArray, callback_coordinates)
    rospy.Subscriber('x_ray_coordinates', Float64MultiArray, callback_x_ray_coords)
    subCurrentCoords = rospy.Subscriber('current_coordinates', Float64MultiArray, getInitialCoordinates)


    dt = 100 # ms

    r = rospy.Rate(1000. / dt)
    while not rospy.is_shutdown():
        coords.data[0] = coords.data[0] + w * dt / 1000.
        coords.data[2] = coords.data[2] + v_x * dt / 1000.
        coords.data[4] = coords.data[4] + v_y * dt / 1000.

        if publish:
            rospy.loginfo("I published. %s" % coords.data)
            pubCoords.publish(coords)

        r.sleep()