#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String
from skimage import io, color, filters, measure
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
import threading

# def callback(data):
#     global s0_pos, s1_pos
#     # Structure of data: [theta, w, pos0, vel0, pos1, vel1] in rad, rad/s, mm, mm/s
#     s0_pos = 10000000.*data.data[2]/24.44 + initial_offset0
#     s0_vel = 5000.*data.data[3]
#     s1_pos = 10000000.*data.data[4]/24.44 + initial_offset1
#     s1_vel = 5000.*data.data[5]
#
#     stepper_pos_vel.data = [data.data[0], data.data[1]]

### mark the calibration pixel for the alignment of the glass petri
def mark_petri_calibration_pixel():
    pass

# class camThread(threading.Thread):
#     def __init__(self, previewName, camPath):
#         threading.Thread.__init__(self)
#         self.previewName = previewName
#         self.camPath = camPath
#     def run(self):
#         print("Starting " + self.previewName)
#         camPreview(self.previewName, self.camPath)

# def camPreview(previewName, camPath):
#     cv2.namedWindow(previewName)
#     cam = cv2.VideoCapture(camPath)
#     if cam.isOpened():  # try to get the first frame
#         rval, frame = cam.read()
#     else:
#         rval = False

#     while rval:
#         cv2.imshow(previewName, frame)
#         rval, frame = cam.read()
#         key = cv2.waitKey(20)
#         if key == 27:  # exit on ESC
#             break
#     cv2.destroyWindow(previewName)

if __name__ == '__main__':

    rospy.init_node('x_ray_position', anonymous=True)
    pubXrayCoords = rospy.Publisher('x_ray_coordinates', Float64MultiArray, queue_size=10)
    # rospy.Subscriber('coordinates', Float64MultiArray, callback)
    xRayCoords = Float64MultiArray()
    xRayCoords.data = [0, 0]

    # cam = input('give device path for camera:')

    # cap0 = cv2.VideoCapture(cam)
    cap0 = cv2.VideoCapture('/dev/video4')
    cap1 = cv2.VideoCapture('/dev/video2')
    # winname0 = cam
    winname0 = 'Screen Capture 0'
    winname1 = 'Screen Capture 1'
    cv2.namedWindow(winname0)        # Create a named window
    cv2.moveWindow(winname0, 1000,30)
    cv2.namedWindow(winname1)        # Create a named window
    cv2.moveWindow(winname1, 10,30)

    i = 0
    dt = 50 # ms
    r = rospy.Rate(1000. / dt)

    # tic = time.clock()

    captured_pics = 0 # number of captured pictures 

    backsub = cv2.createBackgroundSubtractorMOG2()

    while not rospy.is_shutdown():

        ###< read the images from both cameras
        ret0, image0 = cap0.read()
        ret1, image1 = cap1.read()

        ###< rotate the image 90 degrees counterclockwise
        rotated_image0 = cv2.rotate(image0, cv2.ROTATE_90_COUNTERCLOCKWISE)
        rotated_image1 = cv2.rotate(image1, cv2.ROTATE_90_COUNTERCLOCKWISE)

        ###< crop the petri cover part of the image
        cropped_image0 = rotated_image0[235:340, 215:345]
        cropped_image1 = rotated_image1[235:340, 215:345]

        ###< denoised cropped images
        denoised_image0 = cv2.fastNlMeansDenoisingColored(cropped_image0,None,10,10,7,15)
        denoised_image1 = cv2.fastNlMeansDenoisingColored(cropped_image1,None,10,10,7,15)

        ###< convert color space to gray
        gray_image0 = cv2.cvtColor(denoised_image0, cv2.COLOR_BGR2GRAY)
        gray_image1 = cv2.cvtColor(denoised_image1, cv2.COLOR_BGR2GRAY)
        rospy.loginfo('min values')
        rospy.loginfo(gray_image0.min())
        rospy.loginfo(gray_image1.min())
        rospy.loginfo('min coordinates')
        argmin0 = np.argmin(gray_image0)
        shape0 = gray_image0.shape
        argmin1 = np.argmin(gray_image1)
        shape1 = gray_image1.shape
        rospy.loginfo((int(argmin0/shape0[1]), int(argmin0%shape0[1])))
        rospy.loginfo((int(argmin1/shape1[1]), int(argmin1%shape1[1])))
        # rospy.loginfo(gray_image1.argmin())
        _, bin_image0 = cv2.threshold(gray_image0, gray_image0.min() + 20, 255, cv2.THRESH_BINARY)
        _, bin_image1 = cv2.threshold(gray_image1, gray_image1.min() + 20, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5,5),np.uint8)
        bin_image0 = cv2.morphologyEx(bin_image0.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        bin_image1 = cv2.morphologyEx(bin_image1.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        bin_image0 = cv2.morphologyEx(bin_image0.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        bin_image1 = cv2.morphologyEx(bin_image1.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

        contours0, hierarchy = cv2.findContours(bin_image0, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # rospy.loginfo(contours0)
        contours1, hierarchy = cv2.findContours(bin_image1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # rospy.loginfo(contours1)
        robot0_contour = contours0[1]
        # print(robot0_contour)
        robot0_y = (np.min(robot0_contour[:,:,0]) + np.max(robot0_contour[:,:,0]))/2
        robot0_x = (np.min(robot0_contour[:,:,1]) + np.max(robot0_contour[:,:,1]))/2
        rospy.loginfo((robot0_x, robot0_y))

        cv2.drawContours(cropped_image0, contours0, -1, (150,150,150), 2)
        cv2.drawContours(cropped_image1, contours1, -1, (150,150,150), 2)
        
        try: 
            cv2.imshow(winname0, cropped_image0)
        except:
            rospy.loginfo("temassizlik 0")

        try:
            cv2.imshow(winname1, cropped_image1)
        except:
            rospy.loginfo("temassizlik 1")

        if cv2.waitKey(50) & 0xFF == ord('q'):
            rospy.loginfo("kapatmaya bastin")
            break


    # # while(True):

    #     # toc = time.clock()
    #     # print(toc - tic)
    #     # tic = toc
    #     ret0, image0 = cap0.read()
    #     #cap0.release()
    #     ret1, image1 = cap1.read()

    #     ###< rotate the image 90 degrees counterclockwise
    #     rotated_image0 = cv2.rotate(image0, cv2.ROTATE_90_COUNTERCLOCKWISE)
    #     rotated_image1 = cv2.rotate(image1, cv2.ROTATE_90_COUNTERCLOCKWISE)

    #     ###< crop the petri cover part of the image
    #     cropped_image0 = rotated_image0[235:345, 210:350]
    #     cropped_image1 = rotated_image1[235:345, 210:350]

    #     dst0 = cv2.fastNlMeansDenoisingColored(cropped_image0,None,8,8,7,15)

    #     hsv_cropped_image0 = cv2.cvtColor(dst0, cv2.COLOR_BGR2HSV)
    #     hsv_cropped_image0_hue = hsv_cropped_image0[:, :, 0]
    #     hsv_cropped_image1 = cv2.cvtColor(cropped_image1, cv2.COLOR_BGR2HSV)
    #     hsv_cropped_image1_hue = hsv_cropped_image1[:, :, 0]
        
    #     lower = np.array([80,0,0])
    #     upper = np.array([110,100,110])
    #     mask = cv2.inRange(hsv_cropped_image1, lower, upper)

    #     ###< convert color space to gray
    #     gray_image0 = cv2.cvtColor(dst0, cv2.COLOR_BGR2GRAY)
    #     # gray_image1 = cv2.cvtColor(hsv_cropped_image1, cv2.COLOR_BGR2GRAY)

    #     _, imbin0 = cv2.threshold(gray_image0, 135, 255, cv2.THRESH_BINARY)
    #     # imbin0 = cv2.flip(cv2.transpose(imbin0), 1)

    #     # _, imbin1 = cv2.threshold(hsv_cropped_image1, 87, 255, cv2.THRESH_BINARY)
    #     # imbin1 = cv2.flip(cv2.transpose(imbin1), 1)

    #     kernel = np.ones((9, 9),np.uint8)
        
    #     # imbin0 = cv2.morphologyEx(imbin0.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    #     imbin0 = cv2.morphologyEx(imbin0.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    #     # label_img0 = measure.label(imbin0, connectivity=imbin0.ndim)
    #     # props0 = measure.regionprops(label_img0)

    #     contours, hierarchy = cv2.findContours(imbin0, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #     cv2.drawContours(cropped_image0, contours, 0, (150,150,150), 2)
    #     mu = cv2.moments(contours[0])
    #     mc = (mu['m10'] / (mu['m00'] + 1e-5), mu['m01'] / (mu['m00'] + 1e-5))
    #     rospy.loginfo(mc)
        

    #     # imbin1 = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    #     # imbin1 = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    #     # label_img1 = measure.label(imbin1, connectivity=imbin1.ndim)
    #     # props1 = measure.regionprops(label_img1)
        
    #     # hsv_cropped_image1 = cv2.morphologyEx(hsv_cropped_image1.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    #     # hsv_cropped_image1 = cv2.morphologyEx(hsv_cropped_image1.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

    #     # fgMask0 = backsub.apply(gray_image0)
    #     # fgMask1 = backsub.apply(gray_image1)

        
    #     ###< convert color space to hsv
    #     # hsv_image0 = cv2.cvtColor(cropped_image0, cv2.COLOR_BGR2HSV)
    #     # hsv_image1 = cv2.cvtColor(cropped_image1, cv2.COLOR_BGR2HSV)

    #     # imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #     # # thresh = filters.threshold_otsu(imgray)
    #     # ret, imbin = cv2.threshold(imgray[440:590,300:700], 25, 255, cv2.THRESH_BINARY)
    #     # imbin = cv2.flip(cv2.transpose(imbin), 1)

    #     # kernel = np.ones((10,10),np.uint8)
    #     # imbin = cv2.morphologyEx(imbin.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    #     # imbin = cv2.morphologyEx(imbin.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    #     # label_img = measure.label(imbin, connectivity=imbin.ndim)
    #     # props = measure.regionprops(label_img)

    #     # for j in range(len(props)):
    #         # xRayCoords.data = [props[j].centroid[1], props[j].centroid[0]] # [x,y]
    #         # pubXrayCoords.publish(xRayCoords)
    #         # rospy.loginfo("I published. %s" % xRayCoords.data)
    #         # imbin[:, props[j].centroid[1].astype(np.uint8)] = abs(imbin[:, props[j].centroid[1].astype(np.uint8)] - 1)
    #         # imbin[props[j].centroid[0].astype(np.uint8), :] = abs(imbin[props[j].centroid[0].astype(np.uint8), :] - 1)
    #         # print('Centroid\t' + str(props[j].centroid))
    #         # print('Perimeter\t' + str(props[j].perimeter))
    #         # print('Area\t\t' + str(props[j].area))
        
    #     try: 
    #         # cv2.imshow(winname0, cropped_image0)
    #         cv2.imshow(winname0, imbin0)
    #     except:
    #         rospy.loginfo("temassizlik 0")

    #     try:
    #         # cv2.imshow(winname1, cropped_image1)
    #         cv2.imshow(winname1, cropped_image0)
    #     except:
    #         rospy.loginfo("temassizlik 1")

    #     #cv2.imwrite('/home/gulec/hakan_images_auto_20ms/c'+str(captured_pics)+'_left.png',cv2.rotate(image0, cv2.ROTATE_90_COUNTERCLOCKWISE))
    #     #cv2.imwrite('/home/gulec/hakan_images_auto_20ms/c'+str(captured_pics)+'_right.png',cv2.rotate(image0, cv2.ROTATE_90_COUNTERCLOCKWISE))
    #     #captured_pics += 1

    #     # if cv2.waitKey(1) & 0xFF == ord('y'): #save on pressing 'y'
    #     #     rospy.loginfo("bastin") 
    #     #     cv2.imwrite('/home/gulec/hakan_images/c'+str(captured_pics)+'_left.png',cv2.rotate(image0, cv2.ROTATE_90_COUNTERCLOCKWISE))
    #     #     cv2.imwrite('/home/gulec/hakan_images/c'+str(captured_pics)+'_right.png',cv2.rotate(image0, cv2.ROTATE_90_COUNTERCLOCKWISE))
    #     #     captured_pics += 1
    #     if cv2.waitKey(50) & 0xFF == ord('q'):
    #         rospy.loginfo("kapatmaya bastin")
    #         break

        r.sleep()
