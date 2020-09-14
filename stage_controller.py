#!/usr/bin/env python3
from thorpy.comm.discovery import discover_stages
import rospy
from std_msgs.msg import Float64MultiArray, String

def stepperPublisher():
    hello_str = "hello world %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    stepper_pos_vel = Float64MultiArray()
    stepper_pos_vel.data = [stepper_pos, stepper_vel]
    pubStepper.publish(stepper_pos_vel)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global stepper_pos, stepper_vel
    stepper_pos = data.data[0]
    stepper_vel = data.data[1]
    s0_pos = 10000000.*data.data[2]/24.44 + 60000000
    s0_vel = 5000.*data.data[3]
    s1_pos = 10000000.*data.data[4]/24.44 + 60000000
    s1_vel = 5000.*data.data[5]
    s0._set_velparams(0, s0_vel, s0.acceleration)
    s1._set_velparams(0, s1_vel, s1.acceleration)
    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s0_pos)))
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, int(s1_pos)))
    stepperPublisher()
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    global pub, pubStepper
    pub = rospy.Publisher('stepper_trial', String, queue_size=10)
    pubStepper = rospy.Publisher('stepper_go', Float64MultiArray, queue_size=10)

    rospy.Subscriber('coordinates', Float64MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    from thorpy.message import *
    
    stages = list(discover_stages())
    print(stages)
    
    # define stages and ports as global
    global s0, s1, p0, p1
    # check serial number to assign axes to stages.
    if stages[0]._port.serial_number == 45875796:
        s0 = stages[0]
        s1 = stages[1]
    else:
        s0 = stages[1]
        s1 = stages[0]

    p0 = s0._port # serial port of stage 0
    p1 = s1._port # serial port of stage 1

    s0._set_homeparams(10000, 1, s0.home_limit_switch, s0.home_offset_distance)
    s1._set_homeparams(10000, 1, s1.home_limit_switch, s1.home_offset_distance)
    s0._set_velparams(0, 25000, 100000)
    s1._set_velparams(0, 25000, 100000)

    s0.print_state()
    s1.print_state()

    s0.home()
    s1.home()

    p0.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, 60000000))
    p1.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s0._chan_ident, 60000000))


    listener()