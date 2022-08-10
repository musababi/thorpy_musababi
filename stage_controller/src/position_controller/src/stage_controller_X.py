#!/usr/bin/env python3
from thorpy.comm.discovery import discover_stages
import rospy
from std_msgs.msg import Float64MultiArray, String, Bool

def callback(data):
    global s0_pos, initial_offset
    # Structure of data: [pos0, vel0] in mm, mm/s
    s0_pos = 10000000.*data.data[0]/24.44 + initial_offset
    s0_vel = 5000.*data.data[1]
    if s0_vel != s0.max_velocity:
        s0._set_velparams(0, s0_vel, s0.acceleration)
    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s0_pos)))


if __name__ == '__main__':
    from thorpy.message import *
    
    stages = list(discover_stages())
    print(stages)
    
    # define stages and ports as global
    global s0, p0, s0_pos, initial_offset

    # check serial number to assign axes to stages.
    if stages[0]._port.serial_number == 45169034:
        s0 = stages[0]
    p0 = s0._port # serial port of stage 0

    s0._set_homeparams(10000, 0, s0.home_limit_switch, s0.home_offset_distance)
    s0._set_velparams(0, 25000, 500000)

    s0.print_state()

    s0.home()

    initial_offset = int(10000000.*75/24.44)
    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, initial_offset))

    s0_pos = initial_offset

    rospy.init_node('listener', anonymous=True)

    # pubCurrentCoords = rospy.Publisher('current_coordinates', Float64MultiArray, queue_size=10)
    pubMotorStatus = rospy.Publisher('motor_in_motion', Bool, queue_size=10)

    rospy.Subscriber('coordinates', Float64MultiArray, callback)

    currentCoords = Float64MultiArray()
    currentCoords.data = [0]

    in_motion = Bool()

    dt = 5 # ms
    r = rospy.Rate(1000. / dt)
    while not rospy.is_shutdown():
        # currentCoords.data[0] = (s0_pos - initial_offset) * 24.44 / 10000000.
        # rospy.loginfo("Current coordinates: %s" % currentCoords.data)

        # s0.print_state()
        in_motion.data = s0.status_in_motion_forward or s0.status_in_motion_reverse
        # pubCurrentCoords.publish(currentCoords)
        pubMotorStatus.publish(in_motion)

        r.sleep()