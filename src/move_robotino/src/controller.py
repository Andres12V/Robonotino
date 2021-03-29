#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
import cv2 as cv

import cv2.aruco as aruco
import numpy as np
import sys, time, math
import imutils


class Controller:
    def __init__(self):
        rospy.init_node('control')
        self.sub_image = rospy.Subscriber('/my_robotino_urdf/camera/image_raw', Image, self.callback)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(5000)
        self.bridge = CvBridge()
        self.msg = Twist()
        msg1 = Odometry()

    def odom_callback(self, msg1):
        pos_x = round(msg1.pose.pose.position.x, 1)
        pos_y = round(msg1.pose.pose.position.y, 1)
        print('Pos x,y', pos_x, pos_y )

        if aruco_num == 25:
            xd = 3
            yd = -3.0
            K_p = 0.3
            #K_i = 0.0003

            global int_ex
            ex = xd-pos_x
            int_ex = int_ex+ex

            self.msg.linear.y = ex*K_p

            global int_ey
            ey = yd-pos_y
            int_ey = int_ey+ey
            self.msg.linear.x = -ey*K_p
            rospy.loginfo('Looking for ArUco tag #'+str(aruco_num))
        self.pub.publish(self.msg)
        self.rate.sleep()
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv.imshow('Image', cv_image)
            aruco_image = cv_image
            aruco_image = imutils.resize(aruco_image, width=500)
            id_to_find= aruco_num
            marker_size=10 #-[cm]
            #  Get the camera calibration path
            calib_path='/home/andresvergara/robotino_ws/src/Robotino/src/move_robotino/src/Calibpath/'
            camera_matrix= np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
            camera_distortion= np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

            #  180 deg rotation matrix aroud the x axis
            R_flip= np.zeros((3,3), dtype=np.float32)
            R_flip[0,0]=1.0
            R_flip[1,1]=-1.0
            R_flip[2,2]=-1.0

            # Definde the aruco dictionary
            aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
            parameters= aruco.DetectorParameters_create()
            font= cv.FONT_HERSHEY_PLAIN
            #Convert in gray scale
            gray=cv.cvtColor(aruco_image, cv.COLOR_BGR2GRAY)

            # FInd all the aruco markers in the image
            corners, ids, rejected=aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
            if ids != None and ids[0] == id_to_find:
                aruco_image = aruco.drawDetectedMarkers(aruco_image, corners)
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
                try:
                    rvec, tvec=ret[0][0,0,:], ret[1][0,0,:]
                    aruco.drawAxis(aruco_image, camera_matrix, camera_distortion, rvec, tvec, 10)
                    str_position= 'Marker Postion x=%4.0f  y=%4.0f  z=%4.0f'%(tvec[0], tvec[1], tvec[2])
                    cv.putText(aruco_image, str_position, (0,100), font, 1, (0, 255, 0), 2, cv.LINE_AA)
                except:
                    print('No ArUco Detected')

            # Display the frame
            cv.imshow('frame', aruco_image)

        except CvBridgeError as e:
            print(e)
        k = cv.waitKey(1)
        if k == ord('r'):
            self.msg.linear.x = 0
            self.msg.linear.y = 0
            self.msg.angular.z = 0
            rospy.signal_shutdown('shutdown')
            cv.destroyAllWindows()
        self.rate.sleep()

if __name__ == '__main__':
    aruco_num = input('Enter ArUco Marker id: ')
    print(aruco_num)
    int_ex = 0
    int_ey = 0
    kt = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException as e:
        print(e)
