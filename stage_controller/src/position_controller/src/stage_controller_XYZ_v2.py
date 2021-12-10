#!/usr/bin/env python3
from thorpy.comm.discovery import discover_stages
import rospy
from std_msgs.msg import Float64MultiArray, Int64MultiArray, String, Bool
import numpy as np

def callback(data):
    global s0, s1, s2, p0, p1, p2, s0_pos, s1_pos, s2_pos, request_sent, i_need_new_position, set_positions
    # Structure of data: [pos0, vel0, pos1, vel1, pos2, vel2] in mm, mm/s
    s0_pos = -10000000.*data.data[0]/24.44 + initial_offset[0]
    s0_vel = 5000.*data.data[1]
    s1_pos = 10000000.*data.data[2]/24.44 + initial_offset[1]
    s1_vel = 5000.*data.data[3]
    s2_pos = -10000000.*data.data[4]/24.44 + initial_offset[2]
    s2_vel = 5000.*data.data[5]
    set_positions = [data.data[0], data.data[2], data.data[4]]
    if s0_vel != s0.max_velocity:
        s0._set_velparams(0, s0_vel, s0.acceleration)
    if s1_vel != s1.max_velocity:
        s1._set_velparams(0, s1_vel, s1.acceleration)
    if s2_vel != s2.max_velocity:
        s2._set_velparams(0, s2_vel, s2.acceleration)
    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s0_pos)))
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s1._chan_ident, int(s1_pos)))
    p2.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s2._chan_ident, int(s2_pos)))

    # request_sent = True
    i_need_new_position = True


if __name__ == '__main__':
    from thorpy.message import *
    
    stages = list(discover_stages())
    print(stages)
    
    # define stages and ports as global
    global s0, s1, s2, p0, p1, p2, s0_pos, s1_pos, s2_pos, request_sent, i_need_new_position, set_positions

    # check serial number to assign axes to stages.
    if stages[0]._port.serial_number == 45875785:
        if stages[1]._port.serial_number == 45235714:
            s0 = stages[0]
            s1 = stages[1]
            s2 = stages[2]
        else:
            s0 = stages[0]
            s1 = stages[2]
            s2 = stages[1]
    elif stages[0]._port.serial_number == 45235714:
        if stages[1]._port.serial_number == 45875785:
            s0 = stages[1]
            s1 = stages[0]
            s2 = stages[2]
        else:
            s0 = stages[2]
            s1 = stages[0]
            s2 = stages[1]
    else:
        if stages[1]._port.serial_number == 45235714:
            s0 = stages[2]
            s1 = stages[1]
            s2 = stages[0]
        else:
            s0 = stages[1]
            s1 = stages[2]
            s2 = stages[0]
    p0 = s0._port # serial port of stage 0
    p1 = s1._port # serial port of stage 1
    p2 = s2._port # serial port of stage 2

    s0._set_homeparams(20000, 0, s0.home_limit_switch, s0.home_offset_distance)
    s1._set_homeparams(20000, 0, s1.home_limit_switch, s1.home_offset_distance)
    s2._set_homeparams(5000, 0, s2.home_limit_switch, s2.home_offset_distance)
    
    # Initialize maximum velocity and accelerations
    #                    vel    acc
    s0._set_velparams(0, 25000, 500000)
    s1._set_velparams(0, 25000, 500000)
    s2._set_velparams(0, 10000, 50000)

    s0.print_state()
    s1.print_state()
    s2.print_state()

    s0.home()
    s1.home()
    s2.home()

    # Initial offset of stages:     -X-                      -Y-                     -Z-
    initial_offset = [int(10000000.*150/24.44), int(10000000.*75/24.44), int(10000000.*65/24.44)]
    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, initial_offset[0]))
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s1._chan_ident, initial_offset[1]))
    p2.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s2._chan_ident, initial_offset[2]))

    s0_pos = initial_offset[0]
    s1_pos = initial_offset[1]
    s2_pos = initial_offset[2]

    rospy.init_node('listener', anonymous=True)

    pubCurrentCoords = rospy.Publisher('current_coordinates', Float64MultiArray, queue_size=10)
    pubSendNextPosition = rospy.Publisher('send_next_position', Bool, queue_size=10)

    rospy.Subscriber('coordinates', Float64MultiArray, callback)

    currentCoords = Float64MultiArray()
    currentCoords.data = [0, 0, 0, 0, 0, 0]
    set_positions = [0, 0, 0]

    msgSendNextPosition = Bool()
    request_sent = False
    i_need_new_position = True

    dt = 20 # ms
    r = rospy.Rate(1000. / dt)
    while not rospy.is_shutdown():
        currentCoords.data[0] = -s0.position / 2045.827 + initial_offset[0] * 24.44 / 10000000.
        currentCoords.data[1] = s1.position / 2045.827 - initial_offset[1] * 24.44 / 10000000.
        currentCoords.data[2] = -s2.position / 2045.827 + initial_offset[2] * 24.44 / 10000000.
        # currentCoords.data[3:] = set_positions
        # rospy.loginfo("Current coordinates: %s" % currentCoords.data)

        # s0.print_state()
        # in_motion.data = s0.status_in_motion_forward or s0.status_in_motion_reverse or s1.status_in_motion_forward or s1.status_in_motion_reverse or s2.status_in_motion_forward or s2.status_in_motion_reverse
        pubCurrentCoords.publish(currentCoords)
        distance2goal = np.linalg.norm(currentCoords.data[:3] - np.array(set_positions))
        # print(distance2goal)
        currentCoords.data[3] = distance2goal
        # Treshold for next position command
        msgSendNextPosition.data = distance2goal < 0.5 # 0.5 mm
        if i_need_new_position and msgSendNextPosition.data:
            pubSendNextPosition.publish(msgSendNextPosition)
            request_sent = True
            i_need_new_position = False

        r.sleep()