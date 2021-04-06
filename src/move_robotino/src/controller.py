#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
import cv2 as cv
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import cv2.aruco as aruco
import numpy as np
import sys, time, math
import imutils

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])
class Controller:
    def __init__(self):
        rospy.init_node('control')
        # Camera Image
        self.sub_image = rospy.Subscriber('/my_robotino_urdf/camera/image_raw', Image, self.callback)
        self.bridge = CvBridge()
        # Move
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.msg = Twist()
        # Infrared sensors
        self.sub_laser1 = rospy.Subscriber('/my_robotino_urdf/laser/scan', LaserScan, self.ls1_callback)
        self.sub_laser2 = rospy.Subscriber('/my_robotino_urdf/laser1/scan', LaserScan, self.ls2_callback)
        self.sub_laser3 = rospy.Subscriber('/my_robotino_urdf/laser2/scan', LaserScan, self.ls3_callback)
        self.sub_laser4 = rospy.Subscriber('/my_robotino_urdf/laser3/scan', LaserScan, self.ls4_callback)
        self.sub_laser5 = rospy.Subscriber('/my_robotino_urdf/laser4/scan', LaserScan, self.ls5_callback)
        self.sub_laser6 = rospy.Subscriber('/my_robotino_urdf/laser5/scan', LaserScan, self.ls6_callback)
        self.sub_laser7 = rospy.Subscriber('/my_robotino_urdf/laser6/scan', LaserScan, self.ls7_callback)
        self.sub_laser8 = rospy.Subscriber('/my_robotino_urdf/laser7/scan', LaserScan, self.ls8_callback)
        self.sub_laser9 = rospy.Subscriber('/my_robotino_urdf/laser8/scan', LaserScan, self.ls9_callback)
        # Position and orientation of the ArUco Marker
        self.pub_aruco_pose = rospy.Publisher('/aruco_pose', Pose, queue_size=10)
        self.rate = rospy.Rate(5000)

        msg1 = Odometry()
        self.aruco_msg = Pose()

    def odom_callback(self, msg1):
        pos_x = round(msg1.pose.pose.position.x, 1)
        pos_y = round(msg1.pose.pose.position.y, 1)

        global theta

        rot_q = msg1.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        theta_deg = round(theta*(180/np.pi),2)
        print('Pos x,y,theta', pos_x, pos_y, theta_deg )

        if aruco_num == 12:
            Flag_r = Flag1*Flag2*Flag3*Flag4*Flag5*Flag6*Flag7*Flag8*Flag9
            xd = -1.5
            yd = -3.0
            K_p = 1  # For ts = 4 sec

            ex = xd-pos_x
            self.msg.linear.y = (ex*K_p)*Flag_r

            ey = yd-pos_y
            self.msg.linear.x = (-ey*K_p)*Flag_r

            #theta_d = np.arctan2(ey, ex)
            #e_heading = theta_d-theta
            #self.msg.angular.z = np.arctan2(np.sin(e_heading), np.cos(e_heading))

            #rospy.loginfo('Looking for ArUco tag #'+str(aruco_num))

        self.pub.publish(self.msg)
        self.rate.sleep()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv.imshow('Image', cv_image)
            aruco_image = cv_image
            aruco_image = imutils.resize(aruco_image, width=500)
            id_to_find = aruco_num
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
            if ids is not None and ids[0] == id_to_find:
                aruco_image = aruco.drawDetectedMarkers(aruco_image, corners)
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
                try:
                    rvec, tvec=ret[0][0,0,:], ret[1][0,0,:]
                    aruco.drawAxis(aruco_image, camera_matrix, camera_distortion, rvec, tvec, 10)
                    str_position= 'Marker Postion x=%4.0f  y=%4.0f  z=%4.0f'%(tvec[0], tvec[1], tvec[2])
                    cv.putText(aruco_image, str_position, (0,290), font, 1, (0, 255, 255), 1, cv.LINE_AA)
                    #-- Obtain the rotation matrix tag->camera
                    R_ct    = np.matrix(cv.Rodrigues(rvec)[0])
                    R_tc    = R_ct.T
                    #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
                    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
                    #-- Print the marker's attitude respect to camera frame
                    str_attitude = "Marker Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                        math.degrees(yaw_marker))
                    cv.putText(aruco_image, str_attitude, (0, 315), font, 1, (0, 255, 255), 1, cv.LINE_AA)

                    #-- Now get Position and attitude f the camera respect to the marker
                    pos_camera = -R_tc*np.matrix(tvec).T

                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                    cv.putText(aruco_image, str_position, (0, 340), font, 1, (255, 255, 0), 1, cv.LINE_AA)

                    #-- Get the attitude of the camera respect to the frame
                    roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                        math.degrees(yaw_camera))
                    cv.putText(aruco_image, str_attitude, (0, 365), font, 1, (255, 255, 0), 1, cv.LINE_AA)

                except:
                    print('No ArUco Detected')
            # Display the frame
            cv.imshow('frame', aruco_image)
            #cv.imwrite('/home/andresvergara/images_aruco/pics/img15.jpg', aruco_image)
            try:
                pos_ar_x = ( float(tvec[0])-float(pos_camera[0]) )/10.0
                pos_ar_y = ( float(tvec[1])-float(pos_camera[1]) )/10.0
                self.aruco_msg.position.x = pos_ar_x
                self.aruco_msg.position.y = pos_ar_y
                self.aruco_msg.position.z = 0.5
                self.pub_aruco_pose.publish(self.aruco_msg)
            except:
                pass

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

    def ls1_callback(self, msg_ls1):
        global Flag1
        dist1 = round(msg_ls1.ranges[359], 2)
        phi1 = 0+theta
        if dist1<0.3:
            self.msg.linear.y = 0.4*(np.cos(phi1+np.pi/2))
            self.msg.linear.x = 0.4*(np.sin(-phi1+np.pi))
            Flag1 = 0
        else:
            Flag1 = 1
        self.pub.publish(self.msg)
    def ls2_callback(self, msg_ls2):
        global Flag2
        dist2 = round(msg_ls2.ranges[359], 2)
        phi2 = (np.pi/4.5)+theta
        if dist2<=0.3:
            self.msg.linear.y = 0.4*(np.cos(phi2-np.pi/2))
            self.msg.linear.x = 0.4*(np.sin(-phi2-np.pi))
            Flag2 = 0
        else:
            Flag2 = 1
        self.pub.publish(self.msg)
    def ls3_callback(self, msg_ls3):
        global Flag3
        dist3 = round(msg_ls3.ranges[359], 2)
        phi3 = (-np.pi/4.5)+theta
        if dist3<=0.3:
            self.msg.linear.y = 0.4*(np.cos(phi3+np.pi/2))
            self.msg.linear.x = 0.4*(np.sin(-phi3+np.pi))
            Flag3 = 0
        else:
            Flag3 = 1
        self.pub.publish(self.msg)
    def ls4_callback(self, msg_ls4):
        global Flag4
        dist4 = round(msg_ls4.ranges[359], 2)
        phi4 = (np.pi/2.25)+theta
        if dist4<=0.3:
            self.msg.linear.y = 0.4*(np.cos(phi4-np.pi/2))
            self.msg.linear.x = 0.4*(np.sin(-phi4-np.pi))
            Flag4 = 0
        else:
            Flag4 = 1
        self.pub.publish(self.msg)
    def ls5_callback(self, msg_ls5):
        global Flag5
        dist5 = round(msg_ls5.ranges[359], 2)
        phi5 = (-np.pi/2.25)+theta
        if dist5<=0.3:
            self.msg.linear.y = 0.4*(np.cos(phi5+np.pi/2))
            self.msg.linear.x = 0.4*(np.sin(-phi5+np.pi))
            Flag5 = 0
        else:
            Flag5 = 1
        self.pub.publish(self.msg)
    def ls6_callback(self, msg_ls6):
        global Flag6
        dist6 = round(msg_ls6.ranges[359], 2)
        phi6 = (np.pi/1.5)+theta
        if dist6<=0.3:
            self.msg.linear.y = 0.4*(np.cos(phi6+np.pi/2))
            self.msg.linear.x = 0.4*(np.sin(-phi6+np.pi))
            Flag6 = 0
        else:
            Flag6 = 1
        self.pub.publish(self.msg)
    def ls7_callback(self, msg_ls7):
        global Flag7
        dist7 = round(msg_ls7.ranges[359], 2)
        phi7 = (-np.pi/1.5)+theta
        if dist7<=0.3:
            self.msg.linear.y = 0.4*(np.cos(phi7-np.pi/2))
            self.msg.linear.x = 0.4*(np.sin(-phi7+np.pi))
            Flag7 = 0
        else:
            Flag7 = 1
        self.pub.publish(self.msg)
    def ls8_callback(self, msg_ls8):
        global Flag8
        dist8 = round(msg_ls8.ranges[359], 2)
        phi8 = (np.pi/1.125)+theta
        if dist8<=0.3:
            self.msg.linear.y = 0.4*(np.cos(phi8+np.pi/2))
            self.msg.linear.x = 0.4*(np.sin(-phi8+np.pi))
            Flag8 = 0
        else:
            Flag8 = 1
        self.pub.publish(self.msg)
    def ls9_callback(self, msg_ls9):
        global Flag9
        dist9 = round(msg_ls9.ranges[359], 2)
        phi9 = (-np.pi/1.125)+theta
        if dist9<=0.3:
            self.msg.linear.y = 0.4*(np.cos(phi9-np.pi/2))
            self.msg.linear.x = 0.4*(np.sin(-phi9+np.pi))
            Flag9 = 0
        else:
            Flag9 = 1
        self.pub.publish(self.msg)

if __name__ == '__main__':
    aruco_num = input('Enter ArUco Marker id: ')
    Flag1 = 0
    Flag2 = 0
    Flag3 = 0
    Flag4 = 0
    Flag5 = 0
    Flag6 = 0
    Flag7 = 0
    Flag8 = 0
    Flag9 = 0
    theta = 0
    kt = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException as e:
        print(e)
