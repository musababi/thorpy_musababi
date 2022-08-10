#!/usr/bin/env python3
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

def callback(data):
    global v_x, v_y, v_z, coords     # v_z is defined here
    v_x = (1 + 10 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[3]
    v_y = (1 + 10 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[4]
    v_z = (1 + 10 * data.buttons[5] + 2 * data.buttons[4]) * (1 - data.axes[5]) / 2
    # w = (0.2 + 4 * data.buttons[5] + 0.8 * data.buttons[4]) * data.axes[0]

    coords.data[1] = abs(v_x)
    coords.data[3] = abs(v_y)
    coords.data[5] = abs(v_z)

    rospy.loginfo("I updated. %s"%coords.data)


if __name__ == '__main__':
    global pubCoords, coords, dt, publish, subCurrentCoords
    publish = False

    rospy.init_node('joy_controller_new', anonymous=True)
    pubCoords = rospy.Publisher('coordinates_new', Float64MultiArray, queue_size=10)

    coords = Float64MultiArray()
    coords.data = [0,0,0,0,0,0]

    v_x = 0
    v_y = 0
    v_z = 0     # v_z init
    w = 0

    rospy.Subscriber('joy', Joy, callback)
    subCurrentCoords = rospy.Subscriber('current_coordinates_new', Float64MultiArray, getInitialCoordinates)

    dt = 200 # ms
    r = rospy.Rate(1000./dt)
    while not rospy.is_shutdown():
        coords.data[0] = coords.data[0] + v_x * dt/1000.
        coords.data[2] = coords.data[2] + v_y * dt/1000.
        coords.data[4] = coords.data[4] + v_z * dt/1000.

        if publish:
            rospy.loginfo("I published. %s"%coords.data)
            pubCoords.publish(coords)
        
        r.sleep()