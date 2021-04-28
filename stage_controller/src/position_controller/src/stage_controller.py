#!/usr/bin/env python3
from thorpy.comm.discovery import discover_stages
import rospy
from std_msgs.msg import Float64MultiArray, String

def callback(data):
    # Structure of data: [theta, w, pos0, vel0, pos1, vel1] in rad, rad/s, mm, mm/s
    s0_pos = 10000000.*data.data[2]/24.44 + initial_offset0
    s0_vel = 5000.*data.data[3]
    s1_pos = 10000000.*data.data[4]/24.44 + initial_offset1
    s1_vel = 5000.*data.data[5]
    if s0_vel != s0.max_velocity:
        s0._set_velparams(0, s0_vel, s0.acceleration)
    if s1_vel != s1.max_velocity:
        s1._set_velparams(0, s1_vel, s1.acceleration)
    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s0_pos)))
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s1_pos)))

    stepper_pos_vel.data = [data.data[0], data.data[1]]
    pubStepper.publish(stepper_pos_vel)

if __name__ == '__main__':
    from thorpy.message import *
    
    stages = list(discover_stages())
    print(stages)
    
    # define stages and ports as global
    global s0, s1, p0, p1, initial_offset, stepper_pos_vel

    stepper_pos_vel = Float64MultiArray()
    # check serial number to assign axes to stages.
    if stages[1]._port.serial_number == 45875796:
        s0 = stages[0]
        s1 = stages[1]
    else:
        s0 = stages[1]
        s1 = stages[0]

    p0 = s0._port # serial port of stage 0
    p1 = s1._port # serial port of stage 1

    s0._set_homeparams(10000, 0, s0.home_limit_switch, s0.home_offset_distance)
    s1._set_homeparams(10000, 1, s1.home_limit_switch, s1.home_offset_distance)
    s0._set_velparams(0, 25000, 100000)
    s1._set_velparams(0, 25000, 100000)

    s0.print_state()
    s1.print_state()

    s0.home()
    s1.home()

    initial_offset0 = 35000000
    initial_offset1 = 120000000
    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, initial_offset0))
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, initial_offset1))


    rospy.init_node('listener', anonymous=True)
    global pub, pubStepper
    # pub = rospy.Publisher('stepper_trial', String, queue_size=10)
    pubStepper = rospy.Publisher('stepper_go', Float64MultiArray, queue_size=10)

    rospy.Subscriber('coordinates', Float64MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()