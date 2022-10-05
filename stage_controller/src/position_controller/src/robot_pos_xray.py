#!/usr/bin/env python3
from std_msgs.msg import Float64MultiArray
import numpy      as np
import rospy
import cv2
import matplotlib.pyplot as plt

if __name__ == '__main__':

    rospy.init_node('xray_position', anonymous=True)

    first_height_mm = 210

    i = 7

    robot_areas_pixel = []
    robot_height_mm = []

    while i < 15:

        xray_image = cv2.imread('/home/gulec/xray-images/30-0619-05M_0001_092822132502_'+str(i)+'.bmp', 0)
        
        winname = 'X Ray Capture'
        
        cv2.namedWindow(winname)        # Create a named window
        cv2.moveWindow(winname, 1000,30)

        cropped_image = xray_image[100:400, 290:490]

        _, bin_image = cv2.threshold(cropped_image, 172, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3,3),np.uint8)
        bin_image = cv2.morphologyEx(bin_image.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        bin_image = cv2.morphologyEx(bin_image.astype(np.uint8), cv2.MORPH_OPEN, kernel)

        # Find the contour of the robot
        contours, _ = cv2.findContours(bin_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        color_image = cv2.cvtColor(cropped_image, cv2.COLOR_GRAY2BGR) 
        
        # for cnt in contours:
        #     # epsilon = 0.1 * cv2.arcLength(cnt, True)
        #     # approx = cv2.approxPolyDP(cnt, epsilon, True)
        #     rect = cv2.minAreaRect(cnt)
        #     box = cv2.boxPoints(rect)
        #     box = np.int0(box)
        #     cv2.drawContours(color_image,[box],0,(0,0,255),1)
        #     print(box)
        #     # cv2.drawContours(cropped_image, [approx], 0, (0), 2)
        
        M = cv2.moments(contours[1])


        color_image = cv2.cvtColor(cropped_image, cv2.COLOR_GRAY2BGR) 
        cv2.drawContours(color_image, contours, 1, (0,0,255), 2)        

        print(M['m00'])

        robot_areas_pixel.append(M['m00'])
        robot_height_mm.append(first_height_mm + (i - 7) * 5)
        i = i + 1

        # cv2.drawContours(color_image, contours, 1, (0,0,255), 2)

        cv2.imshow(winname, color_image)
        # cv2.imshow(winname0, xray_image)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()
    plt.plot(robot_height_mm, robot_areas_pixel)
    plt.show()