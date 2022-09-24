#!/usr/bin/env python3
from ssl import _create_default_https_context
import rospy
from std_msgs.msg import Float64MultiArray, String
from skimage import io, color, filters, measure
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt
import threading
# coefficient_matrix = np.arange(3*15)
coefficient_matrix = np.array([-1.50989285655239e-05,0.000615531351493019,-6.18483977953721e-06,7.97177809965085e-05,-0.000271968941371223,1.87082373452206e-06,0.0243162061386239,-0.183906194155242,-0.00939470322997917,0.000169061099621628,-0.000431798237621481,-5.27580883782027e-05,-0.00322944193845407,-0.000893445019030405,-0.00205382731111485,-0.0113943273735408,0.0325661433291793,-0.0433396910287489,-0.000124683877162292,0.000532653873812744,-6.70393461061592e-05,-6.73283267887168e-05,-0.000261067570103445,-6.97062497443748e-07,0.0679848086435911,0.164837128972136,0.0111402963184286,8.73076750438504e-05,-0.000490223627916592,0.000197224584371825,-0.00308016212093995,-0.00147069354448392,-0.00188378117509396,0.00568394835408348,-0.0237530892857247,-0.0546372464439498,-7.53532204322584e-05,0.000327617088307584,-6.72118288311539e-05,0.00631894498740147,0.00227852098161055,0.00381109433349385,-14.2015745781511,0.675189766499104,-5.55239841991918])
coefficient_matrix = coefficient_matrix.reshape((15,3))
# def callback(data):
#     global s0_pos, s1_pos
#     # Structure of data: [theta, w, pos0, vel0, pos1, vel1] in rad, rad/s, mm, mm/s
#     s0_pos = 10000000.*data.data[2]/24.44 + initial_offset0
#     s0_vel = 5000.*data.data[3]
#     s1_pos = 10000000.*data.data[4]/24.44 + initial_offset1
#     s1_vel = 5000.*data.data[5]
#
#     stepper_pos_vel.data = [data.data[0], data.data[1]]

def checkPastPresent(present, past):
    is_consistent = True
    # if ((present[0] - past[0]) ** 2 + (present[1] - past[1]) ** 2 + (present[2] - past[2]) ** 2) > 25.0:
    #     is_consistent = False
    return is_consistent

def append_new_line(file_name, camera, pin_number, robot_x, robot_y):
    """Append given text as a new line at the end of file"""
    # Open the file in append & read mode ('a+')
    with open(file_name, "a+") as file_object:
        # Move read cursor to the start of file.
        file_object.seek(0)
        # If file is not empty then append '\n'
        data = file_object.read(100)
        if len(data) > 0:
            file_object.write("\n")
        # Append text at the end of file
        text_to_append = camera + ' ' + str(pin_number) + ' ' + str(robot_x) + ' ' + str(robot_y)
        file_object.write(text_to_append)


if __name__ == '__main__':

    rospy.init_node('x_ray_position', anonymous=True)
    pubXrayCoords = rospy.Publisher('x_ray_coordinates', Float64MultiArray, queue_size=10)
    # rospy.Subscriber('coordinates', Float64MultiArray, callback)
    xRayCoords = Float64MultiArray()
    xRayCoords.data = [0, 0]

    pubTwoCamCoords = rospy.Publisher('two_camera_coordinates', Float64MultiArray, queue_size=10)
    TwoCamCoords = Float64MultiArray()
    TwoCamCoords.data = [0.0, 0.0, 0.0, 0]
    is_contour_detected = False
    prev_real_coords = np.array([0.0, 0.0, 0.0])

    # read_coefficient_matrix()

    # cam = input('give device path for camera:')

    # cap0 = cv2.VideoCapture(cam)
    cap0 = cv2.VideoCapture('/dev/video4')
    cap1 = cv2.VideoCapture('/dev/video0')
    
    cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
    cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
    
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
    
    cap1.set(cv2.CAP_PROP_ZOOM, 2)
    cap1.set(cv2.CAP_PROP_ZOOM, 4)

    # focus = 255  # min: 0, max: 255, increment:5
    # print(cap1.set(28, focus))
     

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

    captured_pics = 0 # number of captured pictures 

    backsub = cv2.createBackgroundSubtractorMOG2()

    pin_number=1

    start_consistency = False

    
    tic = time.time()    

    while not rospy.is_shutdown():
        # tic = time.time()    
        ###< read the images from both cameras
        ret0, image0 = cap0.read()
        ret1, image1 = cap1.read()

        toc = time.time()
        # rospy.loginfo('Capture time in sec: %2.4f', toc - tic)
        tic = toc
        
        
        ###< rotate the image 90 degrees counterclockwise
        rotated_image0 = cv2.rotate(image0, cv2.ROTATE_90_COUNTERCLOCKWISE)
        rotated_image1 = cv2.rotate(image1, cv2.ROTATE_90_COUNTERCLOCKWISE)

        ###< crop the petri cover part of the image
        cropped_image0 = rotated_image0[325:530, 290:595]
        cropped_image1 = rotated_image1[330:535, 245:550]

        # tic = time.time()

        ###< denoised cropped images
        # denoised_image0 = cv2.fastNlMeansDenoisingColored(cropped_image0,None,10,10,7,15)
        # denoised_image1 = cv2.fastNlMeansDenoisingColored(cropped_image1,None,10,10,7,15)

        # toc = time.time()
        # rospy.loginfo('Period in sec: %2.4f', toc - tic)

        ###< convert color space to gray
        gray_image0 = cv2.cvtColor(cropped_image0, cv2.COLOR_BGR2GRAY)
        gray_image1 = cv2.cvtColor(cropped_image1, cv2.COLOR_BGR2GRAY)
        # rospy.loginfo('min values')
        # rospy.loginfo(gray_image0.min())
        # rospy.loginfo(gray_image1.min())
        # rospy.loginfo('min coordinates')
        argmin0 = np.argmin(gray_image0)
        shape0 = gray_image0.shape
        argmin1 = np.argmin(gray_image1)
        shape1 = gray_image1.shape
        # rospy.loginfo((int(argmin0/shape0[1]), int(argmin0%shape0[1])))
        # rospy.loginfo((int(argmin1/shape1[1]), int(argmin1%shape1[1])))
        # rospy.loginfo(gray_image1.argmin())
        _, bin_image0 = cv2.threshold(gray_image0, gray_image0.min() + 25, 255, cv2.THRESH_BINARY)
        _, bin_image1 = cv2.threshold(gray_image1, gray_image1.min() + 25, 255, cv2.THRESH_BINARY)

        # tic = time.time()

        kernel = np.ones((3,3),np.uint8)
        bin_image0 = cv2.morphologyEx(bin_image0.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        bin_image1 = cv2.morphologyEx(bin_image1.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        bin_image0 = cv2.morphologyEx(bin_image0.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        bin_image1 = cv2.morphologyEx(bin_image1.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

        # toc = time.time()
        # rospy.loginfo('Period in sec: %2.4f', toc - tic)

        # tic = time.time()

        try:
            contours0, hierarchy = cv2.findContours(bin_image0, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # rospy.loginfo(contours0)
            contours1, hierarchy = cv2.findContours(bin_image1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # rospy.loginfo(contours1)
            robot0_contour = contours0[1]
            robot0_width = np.max(robot0_contour[:,:,0]) - np.min(robot0_contour[:,:,0])
            x0 = (np.max(robot0_contour[:,:,0]) + np.min(robot0_contour[:,:,0])) / 2
            y0 = np.max(robot0_contour[:,:,1]) - robot0_width / 2
            # # print(robot0_contour)
            # y0 = (np.min(robot0_contour[:,:,0]) + np.max(robot0_contour[:,:,0]))/2
            # x0 = (np.min(robot0_contour[:,:,1]) + np.max(robot0_contour[:,:,1]))/2
            # # rospy.loginfo((x0, y0))

            robot1_contour = contours1[1]
            robot1_width = np.max(robot1_contour[:,:,0]) - np.min(robot1_contour[:,:,0])
            x1 = (np.min(robot1_contour[:,:,0]) + np.max(robot1_contour[:,:,0])) / 2
            y1 = np.max(robot1_contour[:,:,1]) - robot1_width / 2
            # # print(robot0_contour)
            # y1 = (np.min(robot1_contour[:,:,0]) + np.max(robot1_contour[:,:,0]))/2
            # x1 = (np.min(robot1_contour[:,:,1]) + np.max(robot1_contour[:,:,1]))/2
            # # rospy.loginfo((x1, y1))
            X_matrix = np.array([x0*x1, x0*x0, x0, y0*x1, y0*y0, y0, x1*y1, x1*x1, x1, x0*y0, y1*y1, y1, x0*y1, y0*y1, 1], dtype=float)
            # print(X_matrix)
            real_coordinates = np.matmul(X_matrix, coefficient_matrix)

            

            if start_consistency == False:
                prev_real_coords[0] = real_coordinates[0]
                prev_real_coords[1] = real_coordinates[1]
                prev_real_coords[2] = real_coordinates[2]
                start_consistency = True
            
            if start_consistency == True:
                is_contour_detected = checkPastPresent(real_coordinates, prev_real_coords)

            if is_contour_detected == True and (cv2.contourArea(robot0_contour) > 400.0 or cv2.contourArea(robot1_contour) > 400.0 or cv2.contourArea(robot0_contour) < 50.0 or cv2.contourArea(robot1_contour) < 50.0):
                is_contour_detected = False 

            if is_contour_detected == True:
                p = 0.2
                rospy.loginfo('Real coordinates updated!')
                prev_real_coords[0] = (1-p)*prev_real_coords[0] + p*real_coordinates[0]
                prev_real_coords[1] = (1-p)*prev_real_coords[1] + p*real_coordinates[1]
                prev_real_coords[2] = (1-p)*prev_real_coords[2] + p*real_coordinates[2]

            print(real_coordinates)
            TwoCamCoords.data[0] = prev_real_coords[0]
            TwoCamCoords.data[1] = prev_real_coords[1]
            TwoCamCoords.data[2] = prev_real_coords[2]
            TwoCamCoords.data[3] = is_contour_detected

            pubTwoCamCoords.publish(TwoCamCoords)    
            # rospy.loginfo('Contour Area - 0: %3.2f', cv2.contourArea(robot0_contour))
            # rospy.loginfo('Contour Area - 1: %3.2f', cv2.contourArea(robot1_contour))        
            rospy.loginfo(checkPastPresent(real_coordinates, prev_real_coords))
            cv2.drawContours(cropped_image0, contours0, 1, (150,150,150), 2)
            cv2.drawContours(cropped_image1, contours1, 1, (150,150,150), 2)
        except Exception as e:
            rospy.logerr(e)
            rospy.loginfo("couldnt detect contours")
            is_contour_detected = 0.0
            TwoCamCoords.data[0] = 0.0
            TwoCamCoords.data[1] = 0.0
            TwoCamCoords.data[2] = 0.0
            TwoCamCoords.data[3] = is_contour_detected
            pubTwoCamCoords.publish(TwoCamCoords)
            pass

        # toc = time.time()
        # rospy.loginfo('Period in sec: %2.4f', toc - tic)

        
        # tic = time.time()

        try: 
            cv2.imshow(winname0, cropped_image0)
        except:
            rospy.loginfo("temassizlik 0")

        try:
            cv2.imshow(winname1, cropped_image1)
        except:
            rospy.loginfo("temassizlik 1")
        
        if cv2.waitKey(5) & 0xFF == ord('q'):
            rospy.loginfo("kapatmaya bastin")
            break

        rospy.loginfo('Contour detection: %2.2f', is_contour_detected)
        # # tic = time.time()
        # # if cv2.waitKey(5) & 0xFF == ord('y'): #save on pressing 'y'
        # #     rospy.loginfo("fotoya bastin")
        # #     rospy.loginfo(pin_number)
        # #     rospy.loginfo(np.min(robot0_contour[:,:,0]))
        # #     rospy.loginfo(np.max(robot0_contour[:,:,0]))
        # #     rospy.loginfo(np.min(robot0_contour[:,:,1]))
        # #     rospy.loginfo(np.max(robot0_contour[:,:,1]))
        # #     cropped_image0[int(y0)][int(x0)] = (255, 255, 255)
        # #     cv2.imwrite('/home/gulec/new_data/data3/c'+str(pin_number)+'_left.png',cropped_image0)
        # #     append_new_line('/home/gulec/new_data/data3/cover3.txt', 'l', pin_number, x0, y0)
            
        # #     cropped_image1[int(y1)][int(x1)] = (255, 255, 255)
        # #     cv2.imwrite('/home/gulec/new_data/data3/c'+str(pin_number)+'_right.png',cropped_image1)
        # #     append_new_line('/home/gulec/new_data/data3/cover3.txt', 'r', pin_number, x1, y1)

        #     # pin_number -= 1
        #     # cv2.imwrite('/home/gulec/hakan_images/c'+str(captured_pics)+'_right.png',cv2.rotate(image0, cv2.ROTATE_90_COUNTERCLOCKWISE))
            
        # toc = time.time()
        # rospy.loginfo('Period in sec: %2.4f', toc - tic)
        # # toc = time.time()
        # # rospy.loginfo('Period in sec: %2.4f', toc - tic)
        # # tic = toc

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
