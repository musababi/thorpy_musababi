#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String, Bool
from pygcode import Line, gcodes
import numpy as np

def callback(data):
    global finished, lines, i, currentCoords, x0, x1, y0, y1, z0, z1

    pubCoords.publish(currentCoords)
    if i < len(lines) - 1:
        i = i + 1
    else:
        finished = True

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
            v = int(f[1:len(f)]) / 60.0
        elif type(g) == gcodes.GCodeLinearMove or type(g) == gcodes.GCodeRapidMove:
            coord_dict = g.get_param_dict()
            x1 = coord_dict['X']
            y1 = coord_dict['Y']
            z1 = coord_dict['Z']
            dx = x1 - x0
            dy = y1 - y0
            dz = z1 - z0
            ds = np.linalg.norm([dx, dy, dz])
            if ds == 0:
                currentCoords.data = [0, 0, 0, 0, 0, 0]
            else:
                currentCoords.data[0] = x1
                currentCoords.data[1] = abs(dx * v / ds)
                currentCoords.data[2] = y1
                currentCoords.data[3] = abs(dy * v / ds)
                currentCoords.data[4] = z1
                currentCoords.data[5] = abs(dz * v / ds)

            x0 = x1
            y0 = y1
            z0 = z1

        elif type(g) == gcodes.GCodeGotoPredefinedPosition:
            currentCoords.data = [0, 10, 0, 10, 0, 10]

if __name__ == '__main__':
    global finished, lines, i, currentCoords, x0, x1, y0, y1, z0, z1
    finished = False

    rospy.init_node('gCode_parser', anonymous=True)

    pubCoords = rospy.Publisher('coordinates', Float64MultiArray, queue_size=10)

    rospy.Subscriber('motor_in_motion', Bool, callback)

    fh = open('/home/mugurlu/Downloads/test.gcode', 'r')
    lines = fh.readlines()

    currentCoords = Float64MultiArray()
    currentCoords.data = np.zeros(6)

    x0 = 0
    y0 = 0
    z0 = 0
    i = 0

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
            v = int(f[1:len(f)]) / 60.0
        elif type(g) == gcodes.GCodeLinearMove or type(g) == gcodes.GCodeRapidMove:
            coord_dict = g.get_param_dict()
            x1 = coord_dict['X']
            y1 = coord_dict['Y']
            z1 = coord_dict['Z']
            dx = x1 - x0
            dy = y1 - y0
            dz = z1 - z0
            ds = np.linalg.norm([dx, dy, dz])
            if ds == 0:
                currentCoords.data = [0, 0, 0, 0, 0, 0]
            else:
                currentCoords.data[0] = x1
                currentCoords.data[1] = abs(dx * v / ds)
                currentCoords.data[2] = y1
                currentCoords.data[3] = abs(dy * v / ds)
                currentCoords.data[4] = z1
                currentCoords.data[5] = abs(dz * v / ds)

            x0 = x1
            y0 = y1
            z0 = z1

        elif type(g) == gcodes.GCodeGotoPredefinedPosition:
            currentCoords.data = [0, 10, 0, 10, 0, 10]

    dt = 500 # ms
    r = rospy.Rate(1000. / dt)
    while not rospy.is_shutdown():
        if finished:
            break

        r.sleep()
