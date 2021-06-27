#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String, Bool
from pygcode import Line, gcodes

def callback(data):
    global in_motion
    in_motion = data.data

if __name__ == '__main__':
    global in_motion
    in_motion = True
    finished = False

    rospy.init_node('gCode_parser', anonymous=True)

    pubCoords = rospy.Publisher('coordinates', Float64MultiArray, queue_size=10)

    rospy.Subscriber('motor_in_motion', Bool, callback)

    fh = open('/home/mugurlu/Downloads/test.gcode', 'r')
    lines = fh.readlines()
    currentCoords = Float64MultiArray()
    currentCoords.data = [0,0]

    dt = 250 # ms
    r = rospy.Rate(1000. / dt)
    i = 0
    while not rospy.is_shutdown():

        if not in_motion and not finished:
            rospy.loginfo("line number %s" % i)
            line = Line(lines[i])

            # print(line)  # will print the line (with cosmetic changes)
            # print(line.block.gcodes)  # is your list of gcodes
            l = sorted(line.block.gcodes)
            for g in l:
                print(str(g))
                print(type(g))
                print(g.get_param_dict())
                if type(g) == gcodes.GCodeFeedRate:
                    f = str(g)
                    currentCoords.data[1] = int(f[1:len(f)])/60.0
                elif type(g) == gcodes.GCodeLinearMove or type(g) == gcodes.GCodeRapidMove:
                    coord_dict = g.get_param_dict()
                    currentCoords.data[0] = coord_dict['X']
                elif type(g) == gcodes.GCodeGotoPredefinedPosition:
                    currentCoords.data = [0, 10]

            # currentCoords.data = [10 * (i%2), 5]
            pubCoords.publish(currentCoords)

            if i < len(lines)-1:
                i = i + 1
            else:
                finished = True
                break

        r.sleep()
