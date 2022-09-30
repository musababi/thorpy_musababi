#!/usr/bin/env python3
from std_msgs.msg import Float64MultiArray
import numpy      as np
import rospy
import cv2

if __name__ == '__main__':

    rospy.init_node('xray_position', anonymous=True)
    # # two_camera_coordinates topic is to publish the real-life position of the robot from robot_pos_2_cams node
    # pubTwoCamCoords     = rospy.Publisher('two_camera_coordinates', Float64MultiArray, queue_size=10)
    # TwoCamCoords        = Float64MultiArray()
    
    # # The last element is to send the contour detection flag
    # TwoCamCoords.data   = [0.0, 0.0, 0.0, 0]
    # is_contour_detected = False
    # prev_real_coords    = np.array([0.0, 0.0, 0.0])         # previous coordinates for each loop

    # # Capture RGB camera images from USB cameras
    # cap0 = cv2.VideoCapture('/dev/video4')
    # cap1 = cv2.VideoCapture('/dev/video0')
    
    # # Set the resolution to 1024 * 768
    # cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
    # cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
    
    # cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
    # cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
    
    xray_image = cv2.imread('/home/gulec/xray-videos/C_Arm_Images/30-0619-05M_0001_20220928132502/7/30-0619-05M_0001_092822132502_7.bmp', 0)
    
    winname = 'X Ray Capture'
    # winname0 = 'X Ray Capt'

    cv2.namedWindow(winname)        # Create a named window
    cv2.moveWindow(winname, 1000,30)


    # cv2.namedWindow(winname0)        # Create a named window
    # cv2.moveWindow(winname0, 1000,30)
    cropped_image = xray_image[100:800, 190:890]

    _, bin_image = cv2.threshold(cropped_image, 170, 255, cv2.THRESH_BINARY)

    kernel = np.ones((3,3),np.uint8)
    bin_image = cv2.morphologyEx(bin_image.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    bin_image = cv2.morphologyEx(bin_image.astype(np.uint8), cv2.MORPH_OPEN, kernel)

    # Find the contour of the robot
    contours, hierarchy = cv2.findContours(bin_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    color_image = cv2.cvtColor(cropped_image, cv2.COLOR_GRAY2BGR) 
            
    for cnt in contours:
        # epsilon = 0.1 * cv2.arcLength(cnt, True)
        # approx = cv2.approxPolyDP(cnt, epsilon, True)
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(color_image,[box],0,(0,0,255),1)
        print(box)
        # cv2.drawContours(cropped_image, [approx], 0, (0), 2)
    
    cv2.imshow(winname, color_image)
    # cv2.imshow(winname0, xray_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # winname0 = 'Screen Capture 0'
    # winname1 = 'Screen Capture 1'
    # cv2.namedWindow(winname0)        # Create a named window
    # cv2.moveWindow(winname0, 1000,30)
    # cv2.namedWindow(winname1)        # Create a named window
    # cv2.moveWindow(winname1, 10,30)

    # dt = 50 # ms
    # r  = rospy.Rate(1000. / dt)

    # start_consistency = False       # Check if the node starts and coordinates are found

    # while not rospy.is_shutdown():
            
    #     # Read the images from both cameras
    #     _, image0 = cap0.read()
    #     _, image1 = cap1.read()

    #     # Rotate the images 90 degrees counterclockwise
    #     rotated_image0 = cv2.rotate(image0, cv2.ROTATE_90_COUNTERCLOCKWISE)
    #     rotated_image1 = cv2.rotate(image1, cv2.ROTATE_90_COUNTERCLOCKWISE)

    #     # Crop the petri cover part of the image
    #     cropped_image0 = rotated_image0[325:530, 290:595]
    #     cropped_image1 = rotated_image1[330:535, 245:550]

    #     # Convert color space to gray
    #     gray_image0 = cv2.cvtColor(cropped_image0, cv2.COLOR_BGR2GRAY)
    #     gray_image1 = cv2.cvtColor(cropped_image1, cv2.COLOR_BGR2GRAY)
        
    #     # Apply binary threshold to grayscale images 
    #     _, bin_image0 = cv2.threshold(gray_image0, gray_image0.min() + 25, 255, cv2.THRESH_BINARY)
    #     _, bin_image1 = cv2.threshold(gray_image1, gray_image1.min() + 25, 255, cv2.THRESH_BINARY)

    #     # Apply morphological open and close operations to reduce noise
    #     kernel = np.ones((3,3),np.uint8)
    #     bin_image0 = cv2.morphologyEx(bin_image0.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    #     bin_image1 = cv2.morphologyEx(bin_image1.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    #     bin_image0 = cv2.morphologyEx(bin_image0.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    #     bin_image1 = cv2.morphologyEx(bin_image1.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

    #     try:
    #         # Find the contour of the robot
    #         contours0, hierarchy = cv2.findContours(bin_image0, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #         contours1, hierarchy = cv2.findContours(bin_image1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
    #         # Find the center of mass of the robot using contour coordinates
    #         robot0_contour = contours0[1]
    #         robot0_width   = np.max(robot0_contour[:,:,0]) - np.min(robot0_contour[:,:,0])
    #         x0 = (np.max(robot0_contour[:,:,0]) + np.min(robot0_contour[:,:,0])) / 2
    #         y0 =  np.max(robot0_contour[:,:,1]) - robot0_width / 2
            
    #         robot1_contour = contours1[1]
    #         robot1_width   = np.max(robot1_contour[:,:,0]) - np.min(robot1_contour[:,:,0])
    #         x1 = (np.min(robot1_contour[:,:,0]) + np.max(robot1_contour[:,:,0])) / 2
    #         y1 = np.max(robot1_contour[:,:,1]) - robot1_width / 2
            
    #         # Obtain the real-life coordinates of the robot doing matrix multiplication operation 
    #         X_matrix = np.array([x0*x1, x0*x0, x0, y0*x1, y0*y0, y0, x1*y1, x1*x1, x1, x0*y0, y1*y1, y1, x0*y1, y0*y1, 1], dtype=float)
    #         real_coordinates = np.matmul(X_matrix, coefficient_matrix)

    #         # Check if the contours are detected using comparison and contour area limitations
    #         if start_consistency == False:
    #             prev_real_coords[0] = real_coordinates[0]
    #             prev_real_coords[1] = real_coordinates[1]
    #             prev_real_coords[2] = real_coordinates[2]
    #             start_consistency = True
            
    #         if start_consistency == True:
    #             is_contour_detected = checkPastPresent(real_coordinates, prev_real_coords)

    #         if is_contour_detected == True and (cv2.contourArea(robot0_contour) > 400.0 or cv2.contourArea(robot1_contour) > 400.0 or cv2.contourArea(robot0_contour) < 50.0 or cv2.contourArea(robot1_contour) < 50.0):
    #             is_contour_detected = False 

    #         if is_contour_detected == True:
    #             p = 0.2         # coefficient for moving averages
                
    #             prev_real_coords[0] = (1-p) * prev_real_coords[0] + p * real_coordinates[0]
    #             prev_real_coords[1] = (1-p) * prev_real_coords[1] + p * real_coordinates[1]
    #             prev_real_coords[2] = (1-p) * prev_real_coords[2] + p * real_coordinates[2]

    #         # Publish the real-life coordinates
    #         TwoCamCoords.data[0] = prev_real_coords[0]
    #         TwoCamCoords.data[1] = prev_real_coords[1]
    #         TwoCamCoords.data[2] = prev_real_coords[2]
    #         TwoCamCoords.data[3] = is_contour_detected

    #         pubTwoCamCoords.publish(TwoCamCoords)    
                
    #         rospy.loginfo('I published: %2.2f, %2.2f, %2.2f', prev_real_coords[0], prev_real_coords[1], prev_real_coords[2])

    #         # Show contour of the robot on the real-time camera image
    #         cv2.drawContours(cropped_image0, contours0, 1, (150,150,150), 2)
    #         cv2.drawContours(cropped_image1, contours1, 1, (150,150,150), 2)
        
    #     except Exception as e:
    #         # Process if the contours could not get detected
    #         rospy.logerr(e)
    #         rospy.loginfo("Could not detect contours")
    #         is_contour_detected  = 0.0
    #         TwoCamCoords.data[0] = 0.0
    #         TwoCamCoords.data[1] = 0.0
    #         TwoCamCoords.data[2] = 0.0
    #         TwoCamCoords.data[3] = is_contour_detected
    #         pubTwoCamCoords.publish(TwoCamCoords)

    #     try: 
    #         cv2.imshow(winname0, cropped_image0)
    #     except:
    #         rospy.loginfo("The images for camera 0 could not shown")

    #     try:
    #         cv2.imshow(winname1, cropped_image1)
    #     except:
    #         rospy.loginfo("The images for camera 1 could not shown")
        
    #     # Stop the camera node if q is pressed
    #     if cv2.waitKey(5) & 0xFF == ord('q'):
    #         rospy.loginfo("The cameras are shut down")
    #         break

    #     rospy.loginfo('Contour detection: %2.2f', is_contour_detected)

    #     r.sleep()
