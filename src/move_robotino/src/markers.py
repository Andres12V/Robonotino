#!/usr/bin/env python2

import rospy
from time import sleep
import cv2 as cv

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

import numpy as np
import sys, time, math
import imutils


class MarkerBasics(object):
    def __init__(self):
        self.sub_aruco_pose = rospy.Subscriber('/aruco_pose', Pose, self.callback)
        self.marker_objectlisher = rospy.Publisher('/marker_basic', Marker, queue_size=10)
        self.rate  = rospy.Rate(1000)
        self.init_marker(index=0, z_val=0.5)

    def callback(self, msg):
        self.msg = Pose()
        global y_aruco
        global x_aruco
        x_aruco = msg.position.x
        y_aruco = msg.position.y
        z_aruco = msg.position.z
        print(msg.position.x)
        #self.rate.sleep()

    def init_marker(self, index=0, z_val=0.5):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = 'base_link'
        self.marker_object.header.stamp = rospy.get_rostime()
        self.marker_object.ns = 'some_robot'
        self.marker_object.id = index
        self.marker_object.type = Marker.CUBE
        self.marker_object.action = Marker.ADD

        my_point = Point()
        my_point.z = z_val
        my_point.y = 0.5
        print(y_aruco)
        my_point.x = 0.3
        self.marker_object.pose.position = my_point

        self.marker_object.pose.orientation.x = 0.0
        self.marker_object.pose.orientation.y = 0.0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0

        self.marker_object.scale.x = 0.05
        self.marker_object.scale.y = 0.05
        self.marker_object.scale.z = 0.05

        self.marker_object.color.r = 0.0
        self.marker_object.color.g = 1.0
        self.marker_object.color.b = 0.0

        self.marker_object.color.a = 1.0

        self.marker_object.lifetime = rospy.Duration(0)

    def start(self):
        while not rospy.is_shutdown():
            self.marker_objectlisher.publish(self.marker_object)
            self.rate.sleep()

if __name__ == '__main__':
    y_aruco = 0.0
    z_aruco = 0.0
    rospy.init_node('marker_basic_node', anonymous=True)
    markerbasics_object = MarkerBasics()
    try:
        markerbasics_object.start()
    except rospy.ROSInterruptException:
        pass
