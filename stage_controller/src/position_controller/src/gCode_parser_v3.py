#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String, Bool
from pygcode import Line, gcodes
import numpy as np
from glob import glob
import re

def callback(data):
    global finished, allCoords

    pubCoords.publish(allCoords)
    finished = True

    

def natural_sort(l, reverse):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key, reverse=reverse)

if __name__ == '__main__':
    global finished, allCoords
    finished = False

    rospy.init_node('gCode_parser', anonymous=True)

    pubCoords = rospy.Publisher('all_coordinates', Float64MultiArray, queue_size=10)

    rospy.Subscriber('send_next_position', Bool, callback)


    path_to_files = "/home/karacakol/Desktop/gcodes"
    gcode_files = glob(path_to_files + "/*.gcode")
    ordered_gcodes = natural_sort(gcode_files, reverse=False)  

    for i in ordered_gcodes:
        print(i)
    

    filename = input("Select file by index:")


    try:
        fh = open(ordered_gcodes[int(filename) - 1], 'r')
        print('Running '+ordered_gcodes[int(filename)-1])
    except:
        fh = open('/home/karacakol/Desktop/gcodes/default.gcode', 'r')
        print('Running default gcode!')
    lines = fh.readlines()

    number_of_lines = len(lines)

    allCoords = Float64MultiArray()
    allCoords.data = np.zeros(6)
    coords = np.zeros(6)

    x0 = 0
    y0 = 0
    z0 = 0
    v = 1

    for i in range(number_of_lines):

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
                try:
                    x1 = coord_dict['X']
                except:
                    x1 = x0
                try:
                    y1 = coord_dict['Y']
                except:
                    y1 = y0
                try:
                    z1 = coord_dict['Z']
                except:
                    z1 = z0
                dx = x1 - x0
                dy = y1 - y0
                dz = z1 - z0
                ds = np.linalg.norm([dx, dy, dz])
                if ds == 0:
                    coords = np.array([x1, 0, y1, 0, z1, 0])
                else:
                    coords[0] = x1
                    coords[1] = abs(dx * v / ds)
                    coords[2] = y1
                    coords[3] = abs(dy * v / ds)
                    coords[4] = z1
                    coords[5] = abs(dz * v / ds)

                x0 = x1
                y0 = y1
                z0 = z1

            elif type(g) == gcodes.GCodeGotoPredefinedPosition:
                coords = np.array([0, 10, 0, 10, 0, 10])

        allCoords.data = np.concatenate((allCoords.data, coords), axis=0)

    print(allCoords.data)
    

    # pubCoords.publish(allCoords)

    dt = 500 # ms
    r = rospy.Rate(1000. / dt)
    while not rospy.is_shutdown():
        if finished:
            break

        r.sleep()
