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


class MarkerBasics():
    def __init__(self):
        rospy.init_node('marker_basic_node', anonymous=True)
        self.sub_aruco_pose = rospy.Subscriber('/aruco_pose', Pose, self.callback)
        self.marker_objectlisher = rospy.Publisher('/marker_basic', Marker, queue_size=10)
        self.rate  = rospy.Rate(1000)

    def callback(self, msg):
        # Get the pose of the aruco marker from the topic
        self.msg = Pose()
        x_aruco = msg.position.x
        y_aruco = msg.position.y
        z_aruco = msg.position.z

        # Define the properties for the marker visualization in Rviz
        self.marker_object = Marker()
        self.marker_object.header.frame_id = 'base_link'
        self.marker_object.header.stamp = rospy.get_rostime()
        self.marker_object.ns = 'some_robot'
        self.marker_object.id = 0
        self.marker_object.type = Marker.CUBE
        self.marker_object.action = Marker.ADD

        my_point = Point()
        my_point.x = x_aruco
        my_point.y = y_aruco
        my_point.z = z_aruco

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
        # Publish the message to the topic
        self.marker_objectlisher.publish(self.marker_object)


if __name__ == '__main__':
    markerbasics_object = MarkerBasics()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
