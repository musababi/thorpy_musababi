#!/usr/bin/env python3
import rospy
from   thorpy.comm.discovery import discover_stages
from   std_msgs.msg          import Float64MultiArray, Bool

def discover_stages():

    from thorpy.comm.port import Port
    from serial.tools.list_ports import comports
    
    serial_ports = [(x[0], x[1], dict(y.split('=', 1) for y in x[2].split(' ') if '=' in y)) for x in comports()]
    
    for ser in serial_ports:
        if 'APT Stepper Motor Controller' in ser[1]:
            p = Port.create(ser[0], ser[2].get('SER', None))
            for stage in p.get_stages().values():
                yield stage

def callback_new(data):
    """Get callback data from joy_with_camera_feedback_controller"""
    global s0_pos, s1_pos, s2_pos
    # Structure of data: [theta, w, pos0, vel0, pos1, vel1] in rad, rad/s, mm, mm/s
    s0_pos = -10000000.*data.data[0]/24.44 + initial_offset[0]
    s0_vel = 5000.*data.data[1]
    s1_pos = 10000000.*data.data[2]/24.44 + initial_offset[1]
    s1_vel = 5000.*data.data[3]
    s2_pos = -10000000.*data.data[4]/24.44 + initial_offset[2]
    s2_vel = 5000.*data.data[5]

    # Send velocity messages to stages
    if s0_vel != s0.max_velocity:
        p0.send_message(MGMSG_MOT_SET_VELPARAMS(chan_ident = s0._chan_ident, min_velocity = int(0), max_velocity = int(s0_vel*4473.92426666), acceleration = 1000000))
    if s1_vel != s1.max_velocity:
        p1.send_message(MGMSG_MOT_SET_VELPARAMS(chan_ident = s1._chan_ident, min_velocity = int(0), max_velocity = int(s1_vel*4473.92426666), acceleration = 1000000))
    if s2_vel != s2.max_velocity:
        p2.send_message(MGMSG_MOT_SET_VELPARAMS(chan_ident = s2._chan_ident, min_velocity = int(0), max_velocity = int(s2_vel*4473.92426666), acceleration = 1000000))

    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s0_pos)))
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s1_pos)))
    p2.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s2_pos)))

    # Send velocity messages to stepper motor algorithm
    stepper_pos_vel.data = [data.data[6], data.data[7], data.data[8], data.data[9]]
    pubStepper.publish(stepper_pos_vel)

if __name__ == '__main__':
    from thorpy.message import *

    print('Here we are!')
    
    stages = list(discover_stages())
    print(stages)
    
    global stepper_pos_vel, s0_pos, s1_pos, s2_pos

    stepper_pos_vel      = Float64MultiArray()
    stepper_pos_vel.data = [0,0,0,0]

    # Define stages and ports as global
    # Check serial number to assign axes to stages.
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

    s0._set_homeparams(10000, 1, s0.home_limit_switch, s0.home_offset_distance)
    s1._set_homeparams(10000, 1, s1.home_limit_switch, s1.home_offset_distance)
    s2._set_homeparams(10000, 1, s2.home_limit_switch, s2.home_offset_distance)
    
    # Initialize maximum velocity and accelerations
    #                    vel    acc
    s0._set_velparams(0, 25000, 1000000)
    s1._set_velparams(0, 25000, 1000000)
    s2._set_velparams(0, 25000, 500000)

    s0.print_state()
    s1.print_state()
    s2.print_state()

    s1.home()
    s0.home()
    s2.home()

    # Initial offset of stages:     -X-                      -Y-                     -Z-
    initial_offset = [int(10000000.*162/24.44), int(10000000.*278/24.44), int(10000000.*35 /24.44)]
    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, initial_offset[0]))
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s1._chan_ident, initial_offset[1]))
    p2.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s2._chan_ident, initial_offset[2]))

    s0_pos = initial_offset[0]
    s1_pos = initial_offset[1]
    s2_pos = initial_offset[2]

    rospy.init_node('listener', anonymous=True)

    # stepper_go topic is to publish the position and velocity of the stepper motors
    pubStepper          = rospy.Publisher('stepper_go', Float64MultiArray, queue_size=10)

    # current_coordinates_new topic is to publish the position of the stages
    pubCurrentCoords    = rospy.Publisher('current_coordinates_new', Float64MultiArray, queue_size=10)

    # coordinates_new topic is to obtain the position of the stages and steppers
    rospy.Subscriber('coordinates_new', Float64MultiArray, callback_new)

    # The xyz stages positions and theta phi array positions
    currentCoords      = Float64MultiArray()
    currentCoords.data = [0,0,0,0,0]

    dt = 100 # ms
    r = rospy.Rate(1000. / dt)

    while not rospy.is_shutdown():
        
        currentCoords.data[0] = (s0_pos - initial_offset[0]) * 24.44 / 10000000.
        currentCoords.data[1] = (s1_pos - initial_offset[1]) * 24.44 / 10000000.
        currentCoords.data[2] = (s2_pos - initial_offset[2]) * 24.44 / 10000000.
        currentCoords.data[3] = stepper_pos_vel.data[0]
        currentCoords.data[4] = stepper_pos_vel.data[2]

        rospy.loginfo("Current coordinates: %s" % currentCoords.data)
        pubCurrentCoords.publish(currentCoords)
        
        r.sleep()