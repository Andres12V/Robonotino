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


class Controller:
    def __init__(self):
        rospy.init_node('control')
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(5000)
        self.bridge = CvBridge()
        self.msg = Twist()
        msg1 = Odometry()



    def odom_callback(self, msg1):
        pos_x = round(msg1.pose.pose.position.x)
        pos_y = round(msg1.pose.pose.position.y)
        print('Pos x,y', pos_x, pos_y )

        if aruco_num == 7:
            xd = 3.0
            yd = -5.0
            K_p = 0.3
            K_i = 0.0003

            global int_ex
            ex = xd-pos_x
            int_ex = int_ex+ex

            self.msg.linear.y = ex*K_p

            global int_ey
            ey = yd-pos_y
            int_ey = int_ey+ey
            self.msg.linear.x = -ey*K_p


            rospy.loginfo('Looking for ArUco tag #'+str(aruco_num))

        else:
            self.msg.linear.x = 0
            self.msg.linear.y = 0
            self.msg.angular.z = 0
        self.pub.publish(self.msg)
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
