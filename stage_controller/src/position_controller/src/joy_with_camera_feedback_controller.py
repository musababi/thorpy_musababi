#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Joy
import numpy as np

def getInitialCoordinates(data):
    global coords, publish
    coords.data[0] = data.data[0]
    coords.data[2] = data.data[1]
    coords.data[4] = data.data[2]
    publish = True
    subCurrentCoords.unregister()

def callback(data):
    global v_x, v_y, v_z, w_x, w_z, r_fb
    v_x = -(1 + 6 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[3]
    v_y = (1 + 6 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[4]
    v_z = (1 + 10 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[1] / 2
    w_x = (data.axes[5] - 1) * data.axes[7] / 4
    w_z = (data.axes[5] - 1) * data.axes[6] / 8     
    r_fb = data.axes[2]

def callback_cam(data):
    global p1, p2, p3, v_x_fb, v_y_fb, v_z_fb, w_x_fb, w_z_fb, p_next 
    p1 = data.data[0]
    p2 = data.data[1]
    p3 = data.data[2]

    p_robot = np.array([p1, p2, p3])
    u_centerline = np.array([-np.sin(coords.data[8]) * np.cos(coords.data[6]), np.cos(coords.data[8]) * np.cos(coords.data[6]), np.sin(coords.data[6])])
    r_SA = np.array([coords.data[0], coords.data[2] + 150, coords.data[4]]) 
    r_c_robot = p_robot - r_SA - np.dot((p_robot - r_SA), u_centerline) * u_centerline
    r_rn = p_next - p_robot 
    theta_next = np.arctan(-r_rn[0] / r_rn[1])
    phi_next = np.arctan( r_rn[2] * np.cos(theta_next) / r_rn[1])
    P_radial = 0.1
    P_angular = 0
    rospy.loginfo('        Center of Array: %3.2f, %3.2f, %3.2f', r_SA[0], r_SA[1], r_SA[2])
    rospy.loginfo('         Robot position: %3.2f, %3.2f, %3.2f', p_robot[0], p_robot[1], p_robot[2])
    rospy.loginfo(' Unit vector Centerline: %3.2f, %3.2f, %3.2f', u_centerline[0], u_centerline[1], u_centerline[2])
    rospy.loginfo('Vector from CL to robot: %3.2f, %3.2f, %3.2f', r_c_robot[0], r_c_robot[1], r_c_robot[2])
    rospy.loginfo('Vector from robot to np: %3.2f, %3.2f, %3.2f', r_rn[0], r_rn[1], r_rn[2])
    rospy.loginfo('   Theta current & next: %2.2f, %2.2f' %(coords.data[8], theta_next))
    rospy.loginfo('     Phi current & next: %2.2f, %2.2f' %(coords.data[6], phi_next))
    
    v_x_fb = r_c_robot[0] * P_radial
    v_y_fb = r_c_robot[1] * P_radial
    v_z_fb = r_c_robot[2] * P_radial
    w_x_fb = (phi_next - coords.data[6]) * P_angular
    w_z_fb = (theta_next - coords.data[8]) * P_angular

    # print(r_c_robot)

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
    v_x_fb = 0
    v_y_fb = 0
    v_z_fb = 0
    w_x_fb = 0
    w_z_fb = 0
    r_fb = 0

    p_next = np.array([0.0, 0.0, 0.0])

    rospy.Subscriber('joy', Joy, callback)
    subCurrentCoords = rospy.Subscriber('current_coordinates_new', Float64MultiArray, getInitialCoordinates)

    rospy.Subscriber('two_camera_coordinates', Float64MultiArray, callback_cam)


    dt = 250 # ms
    r = rospy.Rate(1000./dt)
    while not rospy.is_shutdown():
        coords.data[0] = coords.data[0] + (v_x + v_x_fb) * dt/1000.
        coords.data[2] = coords.data[2] + (v_y + v_y_fb) * dt/1000.
        coords.data[4] = coords.data[4] + (v_z + v_z_fb) * dt/1000.
        coords.data[6] = coords.data[6] + (w_x + w_x_fb) * dt/1000.
        coords.data[8] = coords.data[8] + (w_z + w_z_fb) * dt/1000.
        coords.data[1] = abs(v_x + v_x_fb)
        coords.data[3] = abs(v_y + v_y_fb)
        coords.data[5] = abs(v_z + v_z_fb)
        coords.data[7] = abs(w_x + w_x_fb)
        coords.data[9] = abs(w_z + w_z_fb)
        v_x = v_x + v_x_fb
        v_y = v_y + v_y_fb
        v_z = v_z + v_z_fb
        w_x = w_x + w_x_fb
        w_z = w_z + w_z_fb
        if publish:
            # rospy.loginfo("I published. %s"%coords.data)
            pubCoords.publish(coords)
            rospy.loginfo('   v_z: %.2f', v_z)
            rospy.loginfo('v_z_fb: %.2f', v_z_fb)
        r.sleep()