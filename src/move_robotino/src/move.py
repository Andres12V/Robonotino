#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from time import sleep


rospy.init_node('rotate')

publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rotate_right = True
while not rospy.is_shutdown():
    msg = Twist()
    msg.angular.z = 0.1 if rotate_right else -0.1

    publisher.publish(msg)
    rospy.loginfo('Msg published.')
    rotate_right = not rotate_right
    sleep(10)
