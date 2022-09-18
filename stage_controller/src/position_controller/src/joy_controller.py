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
    subCurrentCoords.unregister()

def callback(data):
    global v_x, v_y, v_z, w_x, w_z, coords     # v_z is defined here
    v_x = -(1 + 6 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[3]
    v_y = (1 + 6 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[4]
    v_z = (1 + 10 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[1] / 2
    # w_x = (0.2 + 4 * data.buttons[5] + 0.8 * data.buttons[4]) * data.axes[7]
    # w_z = (0.2 + 4 * data.buttons[5] + 0.8 * data.buttons[4]) * data.axes[6]
    # acceleration for the steppers is R2 
    w_x = - (data.axes[5] - 1) * data.axes[7] / 4
    w_z = (data.axes[5] - 1) * data.axes[6] / 8            


    coords.data[1] = abs(v_x)
    coords.data[3] = abs(v_y)
    coords.data[5] = abs(v_z)
    coords.data[7] = abs(w_x)
    coords.data[9] = abs(w_z)

    rospy.loginfo("I updated. %s"%coords.data)


if __name__ == '__main__':
    global pubCoords, coords, dt, publish, subCurrentCoords
    publish = False

    rospy.init_node('joy_controller_new', anonymous=True)
    pubCoords = rospy.Publisher('coordinates_new', Float64MultiArray, queue_size=10)

    coords = Float64MultiArray()
    coords.data = [0,0,0,0,0,0,0,0,0,0]
    # initialize velocities
    v_x = 0
    v_y = 0
    v_z = 0  
    w_x = 0
    w_z = 0

    rospy.Subscriber('joy', Joy, callback)
    subCurrentCoords = rospy.Subscriber('current_coordinates_new', Float64MultiArray, getInitialCoordinates)

    dt = 250 # ms
    r = rospy.Rate(1000./dt)
    while not rospy.is_shutdown():
        coords.data[0] = coords.data[0] + v_x * dt/1000.
        coords.data[2] = coords.data[2] + v_y * dt/1000.
        coords.data[4] = coords.data[4] + v_z * dt/1000.
        coords.data[6] = coords.data[6] + w_x * dt/1000.
        coords.data[8] = coords.data[8] + w_z * dt/1000.
        if publish:
            rospy.loginfo("I published. %s"%coords.data)
            pubCoords.publish(coords)
        
        r.sleep()