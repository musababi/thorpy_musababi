#!/usr/bin/env python3
from thorpy.comm.discovery import discover_stages
import rospy
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Joy

def getInitialCoordinates(data):
    global coords, publish
    coords.data[0] = data.data[0]
    coords.data[2] = data.data[1]
    coords.data[4] = data.data[2]
    publish = True
    # subCurrentCoords.unregister()

def callback_x_ray_coords(data):
    global w, v_x, v_y, v_x_fb, coords
    r_ref = 75
    r_err = r_ref - data.data[0]
    v_x_fb = - r_err * 0.1

    coords.data[1] = abs(w)
    coords.data[3] = abs(v_x + v_x_fb)
    coords.data[5] = abs(v_y)

def callback(data):
    global w, v_x, v_y, coords
    v_x = (1 + 10 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[3]
    v_y = (1 + 10 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[4]
    w = (0.2 + 4 * data.buttons[5] + 0.8 * data.buttons[4]) * data.axes[0]

    rospy.loginfo("Joy velocities %s"%[w, v_x, v_y])

if __name__ == '__main__':
    global pubCoords, coords, dt, publish, subCurrentCoords
    publish = False

    rospy.init_node('radial_feedback_controller', anonymous=True)

    coords = Float64MultiArray()
    coords.data = [0,0,0,0,0,0]
    v_x_fb = 0
    v_x = 0
    v_y = 0
    w = 0
    pubCoords = rospy.Publisher('coordinates', Float64MultiArray, queue_size=10)

    rospy.Subscriber('joy', Joy, callback)
    rospy.Subscriber('x_ray_coordinates', Float64MultiArray, callback_x_ray_coords)
    subCurrentCoords = rospy.Subscriber('current_coordinates', Float64MultiArray, getInitialCoordinates)

    dt = 200 # ms

    r = rospy.Rate(1000. / dt)
    while not rospy.is_shutdown():
        coords.data[0] = coords.data[0] + w * dt / 1000.
        coords.data[2] = coords.data[2] + (v_x + v_x_fb) * dt / 1000.
        coords.data[4] = coords.data[4] + v_y * dt / 1000.

        if publish:
            rospy.loginfo("I published. %s" % coords.data)
            pubCoords.publish(coords)

        r.sleep()