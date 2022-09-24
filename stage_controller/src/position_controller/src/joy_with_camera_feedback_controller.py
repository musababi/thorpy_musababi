#!/usr/bin/env python3
from tracemalloc import DomainFilter
import rospy
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Joy
import numpy as np
import fuzzylogic as fl
from fuzzylogic.classes import Domain
import fuzzylogic.functions
import time
from datetime import datetime

# global v_x_max, v_y_max, v_z_max, w_x_max, w_z_max

# #set maximum velocities for each axis
# v_x_max = 1.0
# v_y_max = 1.0
# v_z_max = 1.0
# w_x_max = 1.0
# w_z_max = 1.0

# def setFuzzyLogicForVelocities(current_velocity, feedback_velocity, maximum_value):
#     if abs(current_velocity + feedback_velocity) < maximum_value / 2:
#         current_velocity = current_velocity + feedback_velocity
#     elif abs(current_velocity + feedback_velocity) <  maximum_value:
#         current_velocity = current_velocity + feedback_velocity / 2
#     elif feedback_velocity >= maximum_value:
#         current_velocity = maximum_value
#     return current_velocity    

def append_new_line(file_name, current_time, coords, p1, p2, p3, p_next, radial_on, angular_on, remote_feed, angular_check):
    """Append given text as a new line at the end of file"""
    # Open the file in append & read mode ('a+')
    with open(file_name, "a+") as file_object:
        # Move read cursor to the start of file.
        file_object.seek(0)
        # If file is not empty then append '\n'
        data = file_object.read(100)
        if len(data) > 0:
            file_object.write("\n")
        # Append text at the end of file
        text_to_append = current_time + ' ' + str(coords.data[0]) + ' ' + str(coords.data[2]) + ' ' + str(coords.data[4]) +\
            ' ' + str(coords.data[6]) + ' ' + str(coords.data[8]) + ' ' + str(p1) + ' ' + str(p2) + ' ' + str(p3) + ' ' +\
                str(p_next[0]) + ' ' + str(p_next[1]) + ' ' + str(p_next[2]) + ' ' + str(radial_on) + ' ' + str(angular_on) +\
                    ' ' + str(remote_feed) + ' ' + str(angular_check)
        file_object.write(text_to_append)

def getInitialCoordinates(data):
    global coords, publish
    coords.data[0] = data.data[0]
    coords.data[2] = data.data[1]
    coords.data[4] = data.data[2]
    publish = True
    subCurrentCoords.unregister()

def checkNextPos2Centerline(r_c_next):
    angular_check = 1.0
    if np.linalg.norm(r_c_next) < 1:
        angular_check = 0.0
    return angular_check

def callback(data):
    global v_x, v_y, v_z, w_x, w_z, remote_feed, radial_on, angular_on
    
    remote_feed = data.axes[2]

    if data.buttons[2]:
        radial_on = 0
        angular_on = 0
    if data.buttons[1]:
        radial_on = 1
    if data.buttons[3]:
        angular_on = 1

    if abs(remote_feed - 1.0) < 0.01:
        v_x = -(1 + 6 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[3]
        v_y = (1 + 6 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[4]
        v_z = (1 + 10 * data.buttons[5] + 2 * data.buttons[4]) * data.axes[1] / 2
        w_x = (data.axes[5] - 1) * data.axes[7] / 8
        w_z = (data.axes[5] - 1) * data.axes[6] / 8
    else:
        # rospy.loginfo('Let\'s feed it!')
        direction = 2*data.buttons[0] - 1
        v_x = u_centerline[0] * (1 - remote_feed) * (1 + 6 * data.buttons[5] + 2 * data.buttons[4]) * direction
        v_y = u_centerline[1] * (1 - remote_feed) * (1 + 6 * data.buttons[5] + 2 * data.buttons[4]) * direction
        v_z = u_centerline[2] * (1 - remote_feed) * (1 + 6 * data.buttons[5] + 2 * data.buttons[4]) * direction

def callback_cam(data):
    # tic = time.time()
    global p1, p2, p3, v_x_fb, v_y_fb, v_z_fb, w_x_fb, w_z_fb, p_next, u_centerline, c_d

    p1 = data.data[0]
    p2 = data.data[1]
    p3 = data.data[2]
    c_d = data.data[3]
        
    p_robot = np.array([p1, p2, p3])
    u_centerline = np.array([-np.sin(coords.data[8]) * np.cos(coords.data[6]), np.cos(coords.data[8]) * np.cos(coords.data[6]), np.sin(coords.data[6])])
    r_SA = np.array([coords.data[0], coords.data[2] + 150, coords.data[4]]) 
    r_c_robot = p_robot - r_SA - np.dot((p_robot - r_SA), u_centerline) * u_centerline
    r_c_next = p_next - r_SA - np.dot((p_next - r_SA), u_centerline) * u_centerline
    r_rn = p_next - p_robot 
    theta_next = np.arctan(-r_rn[0] / r_rn[1])
    phi_next = np.arctan( r_rn[2] * np.cos(theta_next) / r_rn[1])
    P_radial = 0.5 * c_d * radial_on
    P_angular = 0.1 * c_d * angular_on * checkNextPos2Centerline(r_c_next)
    rospy.loginfo('        Center of Array: %3.2f, %3.2f, %3.2f', r_SA[0], r_SA[1], r_SA[2])
    rospy.loginfo('         Robot position: %3.2f, %3.2f, %3.2f', p_robot[0], p_robot[1], p_robot[2])
    rospy.loginfo(' Unit vector Centerline: %3.2f, %3.2f, %3.2f', u_centerline[0], u_centerline[1], u_centerline[2])
    rospy.loginfo('Vector from CL to robot: %3.2f, %3.2f, %3.2f', r_c_robot[0], r_c_robot[1], r_c_robot[2])
    rospy.loginfo('Vector from robot to np: %3.2f, %3.2f, %3.2f', r_rn[0], r_rn[1], r_rn[2])
    rospy.loginfo('   Theta current & next: %2.2f, %2.2f' %(coords.data[8], theta_next))
    rospy.loginfo('     Phi current & next: %2.2f, %2.2f' %(coords.data[6], phi_next))
    rospy.loginfo('  CL to next & is close: %2.2f, %2.2f', np.linalg.norm(r_c_next), checkNextPos2Centerline(r_c_next))
    
    v_x_fb = r_c_robot[0] * P_radial
    v_y_fb = r_c_robot[1] * P_radial
    v_z_fb = r_c_robot[2] * P_radial
    w_x_fb = (phi_next - coords.data[6]) * P_angular
    w_z_fb = (theta_next - coords.data[8]) * P_angular
    # toc = time.time()
    # rospy.loginfo('  Total time in seconds: %2.4f', toc - tic)
    # print(r_c_robot)

def getNextPosition(data):
    global p_next
    p_next = np.array(data.data)
    # if np.linalg.norm(r_rn) < 1:
    #     p_next = temp_p_next

if __name__ == '__main__':
    global pubCoords, coords, dt, publish, subCurrentCoords, angular_check
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
    remote_feed = 1.0
    direction = -1
    radial_on = 0
    angular_on = 0
    angular_check = 1.0
    #double check, recently added
    p1 = 0
    p2 = 0
    p3 = 0
    

    rospy.Subscriber('next_position', Float64MultiArray, getNextPosition)

    p_next = np.array([0.0, -15.0, -20.0])

    rospy.Subscriber('joy', Joy, callback)
    subCurrentCoords = rospy.Subscriber('current_coordinates_new', Float64MultiArray, getInitialCoordinates)

    rospy.Subscriber('two_camera_coordinates', Float64MultiArray, callback_cam)


    dt = 20 # ms
    r = rospy.Rate(1000./dt)
    while not rospy.is_shutdown():
        # v_x = setFuzzyLogicForVelocities(v_x, v_x_fb, v_x_max)
        # v_y = setFuzzyLogicForVelocities(v_y, v_y_fb, v_y_max)
        # v_z = setFuzzyLogicForVelocities(v_z, v_z_fb, v_z_max)
        # w_x = setFuzzyLogicForVelocities(w_x, w_x_fb, w_x_max)
        # w_z = setFuzzyLogicForVelocities(w_z, w_z_fb, w_z_max)
        

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
        # coords.data[10] = remote_feed
        # coords.data[11] = direction
        
        if publish:
            # rospy.loginfo("I published. %s"%coords.data)
            pubCoords.publish(coords)
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            append_new_line('/home/gulec/feedback_data/24_09_22.txt', current_time, coords, p1, p2, p3, p_next, radial_on, angular_on, remote_feed, angular_check)
            # rospy.loginfo("Remote feed: %2.2f", remote_feed)
        
        r.sleep()