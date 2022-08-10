#!/usr/bin/env python3
from thorpy.comm.discovery import discover_stages
import rospy
from std_msgs.msg import Float64MultiArray, Int64MultiArray, String, Bool
import numpy as np
# import os
# from thorpy.comm.port import Port
# from serial.tools.list_ports import comports

def discover_stages():
    # import usb
    import os
    from thorpy.comm.port import Port
    from serial.tools.list_ports import comports
    import platform
    
    serial_ports = [(x[0], x[1], dict(y.split('=', 1) for y in x[2].split(' ') if '=' in y)) for x in comports()]
    
    for ser in serial_ports:
        if 'APT Stepper Motor Controller' in ser[1]:
            p = Port.create(ser[0], ser[2].get('SER', None))
            for stage in p.get_stages().values():
                yield stage

# def callback(data):
#     global coords_data, i_got_gCode

#     coords_data = np.array(data.data)
#     for i in coords_data:
#         print('%.3f' % i)
#     i_got_gCode = True

def callback_new(data):
    global s0_pos, s1_pos, s2_pos
    # Structure of data: [theta, w, pos0, vel0, pos1, vel1] in rad, rad/s, mm, mm/s
    s0_pos = 10000000.*data.data[0]/24.44 + initial_offset[0]
    s0_vel = 5000.*data.data[1]
    s1_pos = 10000000.*data.data[2]/24.44 + initial_offset[1]
    s1_vel = 5000.*data.data[3]
    s2_pos = 10000000.*data.data[4]/24.44 + initial_offset[2]
    s2_vel = 5000.*data.data[5]

    if s0_vel != s0.max_velocity:
        s0._set_velparams(0, s0_vel, s0.acceleration)
    if s1_vel != s1.max_velocity:
        s1._set_velparams(0, s1_vel, s1.acceleration)
    if s2_vel != s2.max_velocity:
        s2._set_velparams(0, s2_vel, s2.acceleration)
    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s0_pos)))
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s1_pos)))
    p2.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s2_pos)))

    stepper_pos_vel.data = [data.data[6], data.data[7]]
    pubStepper.publish(stepper_pos_vel)

if __name__ == '__main__':
    from thorpy.message import *
    
    # serial_ports = [(x[0], x[1], dict(y.split('=', 1) for y in x[2].split(' ') if '=' in y)) for x in comports()]
    # stages = []

    # for ser in serial_ports:
    #     print(ser)
    #     if 'APT Stepper Motor Controller' in ser[1]:
    #         p = Port.create(ser[0], ser[2].get('SER', None))
    #         for stage in p.get_stages().values():
    #             stages.append(stage)

    print('Here we are!')
    stages = list(discover_stages())
    print(stages)
    
    global stepper_pos_vel

    stepper_pos_vel = Float64MultiArray()
    stepper_pos_vel.data = [0,0]

    # define stages and ports as global
    # global coords_data, i_got_gCode

    # check serial number to assign axes to stages.
    if stages[0]._port.serial_number == 45875529:
        if stages[1]._port.serial_number == 45875796:
            s0 = stages[0]
            s1 = stages[1]
            s2 = stages[2]
        else:
            s0 = stages[0]
            s1 = stages[2]
            s2 = stages[1]
    elif stages[0]._port.serial_number == 45875796:
        if stages[1]._port.serial_number == 45875529:
            s0 = stages[1]
            s1 = stages[0]
            s2 = stages[2]
        else:
            s0 = stages[2]
            s1 = stages[0]
            s2 = stages[1]
    else:
        if stages[1]._port.serial_number == 45875796:
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

    s0._set_homeparams(10000, 0, s0.home_limit_switch, s0.home_offset_distance)
    s1._set_homeparams(10000, 1, s1.home_limit_switch, s1.home_offset_distance)
    s2._set_homeparams(10000, 1, s2.home_limit_switch, s2.home_offset_distance)
    
    # Initialize maximum velocity and accelerations
    #                    vel    acc
    s0._set_velparams(0, 25000, 1000000)
    s1._set_velparams(0, 25000, 1000000)
    s2._set_velparams(0, 25000, 50000)

    s0.print_state()
    s1.print_state()
    s2.print_state()

    s1.home()
    s0.home()
    s2.home()

    # Initial offset of stages:     -X-                      -Y-                     -Z-
    initial_offset = [int(10000000.*150/24.44), int(10000000.*250/24.44), int(10000000.*100 /24.44)]
    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, initial_offset[0]))
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s1._chan_ident, initial_offset[1]))
    p2.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s2._chan_ident, initial_offset[2]))

    s0_pos = initial_offset[0]
    s1_pos = initial_offset[1]
    s2_pos = initial_offset[2]

    rospy.init_node('listener', anonymous=True)

    pubStepper = rospy.Publisher('stepper_go', Float64MultiArray, queue_size=10)
    pubCurrentCoords = rospy.Publisher('current_coordinates_new', Float64MultiArray, queue_size=10)
    pubSendNextPosition = rospy.Publisher('send_next_position', Bool, queue_size=10)

    # rospy.Subscriber('all_coordinates', Float64MultiArray, callback)

    rospy.loginfo("buradayim")

    rospy.Subscriber('coordinates_new', Float64MultiArray, callback_new)

    currentCoords = Float64MultiArray()
    currentCoords.data = [0, 0, 0, 0]

    # currentCoords = [0, 0, 0]
    set_positions = [0, 0, 0]

    msgSendNextPosition = Bool()
    request_sent = False
    i_got_gCode = False
    i = 0

    dt = 10 # ms
    r = rospy.Rate(1000. / dt)
    while not rospy.is_shutdown():
        currentCoords.data[0] = (s0_pos - initial_offset[0]) * 24.44 / 10000000.
        currentCoords.data[1] = (s1_pos - initial_offset[1]) * 24.44 / 10000000.
        currentCoords.data[2] = (s2_pos - initial_offset[2]) * 24.44 / 10000000.
        currentCoords.data[3] = stepper_pos_vel.data[0]

        rospy.loginfo("Current coordinates: %s" % currentCoords.data)
        # rospy.loginfo("s0 pos: %s" % s0_pos)
        # rospy.loginfo("s1 pos: %s" % s1_pos)
        # rospy.loginfo("s2 pos: %s" % s2_pos)
        pubCurrentCoords.publish(currentCoords)
        # distance2goal = np.linalg.norm(np.array(currentCoords.data) - np.array(set_positions))
        # print('%.4f, [%.4f %.4f %.4f], [%.4f %.4f %.4f]' % (distance2goal, currentCoords[0], currentCoords[1], currentCoords[2], set_positions[0], set_positions[1], set_positions[2]))
        # Treshold for next position command
        # msgSendNextPosition.data = distance2goal < 1 # mm
        # if i_got_gCode and msgSendNextPosition.data:
        #     # Structure of data: [pos0, vel0, pos1, vel1, pos2, vel2] in mm, mm/s
        #     next_coords = coords_data[6*i:6*i+6]
        #     print('%d, [%.3f %.3f %.3f]' %(i+1, next_coords[0], next_coords[2], next_coords[4]))
        #     s0_pos = -10000000.*next_coords[0]/24.44 + initial_offset[0]
        #     s0_vel = 5000.*next_coords[1]
        #     s1_pos = 10000000.*next_coords[2]/24.44 + initial_offset[1]
        #     s1_vel = 5000.*next_coords[3]
        #     s2_pos = -10000000.*next_coords[4]/24.44 + initial_offset[2]
        #     s2_vel = 5000.*next_coords[5]
        #     set_positions = [next_coords[0], next_coords[2], next_coords[4]]
        #     if s0_vel != s0.max_velocity:
        #         s0._set_velparams(0, s0_vel, s0.acceleration)
        #     if s1_vel != s1.max_velocity:
        #         s1._set_velparams(0, s1_vel, s1.acceleration)
        #     if s2_vel != s2.max_velocity:
        #         s2._set_velparams(0, s2_vel, s2.acceleration)
        #     p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s0_pos)))
        #     p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s1._chan_ident, int(s1_pos)))
        #     p2.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s2._chan_ident, int(s2_pos)))

        #     if len(coords_data)/6 > i + 1:
        #         i = i + 1
        #     else:
        #         i_got_gCode = False
        #         i = 0


        r.sleep()