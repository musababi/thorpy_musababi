#!/usr/bin/env python3
from thorpy.comm.discovery import discover_stages
import rospy
from std_msgs.msg import Float64MultiArray, String

def callback(data):
    global s0_pos, s1_pos, s2_pos
    # Structure of data: [pos0, vel0, pos1, vel1, pos2, vel2] in mm, mm/s
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
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s1._chan_ident, int(s1_pos)))
    p2.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s2._chan_ident, int(s2_pos)))


if __name__ == '__main__':
    from thorpy.message import *
    
    stages = list(discover_stages())
    print(stages)
    
    # define stages and ports as global
    global s0, s1, s2, p0, p1, p2, s0_pos, s1_pos, s2_pos

    # check serial number to assign axes to stages.
    if stages[0]._port.serial_number == 45167724:
        if stages[1]._port.serial_number == 45169034:
            s0 = stages[0]
            s1 = stages[1]
            s2 = stages[2]
        else:
            s0 = stages[0]
            s1 = stages[2]
            s2 = stages[1]
    elif stages[0]._port.serial_number == 45169034:
        if stages[1]._port.serial_number == 45167724:
            s0 = stages[1]
            s1 = stages[0]
            s2 = stages[2]
        else:
            s0 = stages[2]
            s1 = stages[0]
            s2 = stages[1]
    else:
        if stages[1]._port.serial_number == 45169034:
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
    s1._set_homeparams(10000, 0, s1.home_limit_switch, s1.home_offset_distance)
    s2._set_homeparams(10000, 0, s2.home_limit_switch, s2.home_offset_distance)
    s0._set_velparams(0, 25000, 500000)
    s1._set_velparams(0, 25000, 500000)
    s2._set_velparams(0, 25000, 500000)

    s0.print_state()
    s1.print_state()
    s2.print_state()

    s0.home()
    s1.home()
    s2.home()

    initial_offset = [int(10000000.*0/24.44), int(10000000.*0/24.44), int(10000000.*0/24.44)]
    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, initial_offset[0]))
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s1._chan_ident, initial_offset[1]))
    p2.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s2._chan_ident, initial_offset[2]))

    s0_pos = initial_offset[0]
    s1_pos = initial_offset[1]
    s2_pos = initial_offset[2]

    rospy.init_node('listener', anonymous=True)

    pubCurrentCoords = rospy.Publisher('current_coordinates', Float64MultiArray, queue_size=10)

    rospy.Subscriber('coordinates', Float64MultiArray, callback)

    currentCoords = Float64MultiArray()
    currentCoords.data = [0, 0, 0]

    dt = 200 # ms
    r = rospy.Rate(1000. / dt)
    while not rospy.is_shutdown():
        currentCoords.data[0] = (s0_pos - initial_offset[0]) * 24.44 / 10000000.
        currentCoords.data[1] = (s1_pos - initial_offset[1]) * 24.44 / 10000000.
        currentCoords.data[2] = (s2_pos - initial_offset[2]) * 24.44 / 10000000.
        # rospy.loginfo("Current coordinates: %s" % currentCoords.data)

        s0.print_state()
        # pubCurrentCoords.publish(currentCoords)

        r.sleep()