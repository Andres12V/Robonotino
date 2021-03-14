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

# import rospy
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge, CvBridgeError
# from time import sleep
# import cv2 as cv
#
# class KeyboardTeleop:
#     def __init__(self):
#         rospy.init_node('keyboard_teleop')
#         self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
#         self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#         self.rate = rospy.Rate(10)
#         self.msg = Twist()
#         self.bridge = CvBridge()
#
#     def callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
#             cv.imshow('Image', cv_image)
#         except CvBridgeError as e:
#             print(e)
#
#         k = cv.waitKey(1)
#
#         if k == ord('w'):
#             self.msg.linear.x = 0.2
#             self.msg.angular.x = 0
#             rospy.loginfo(' w pressed')
#         elif k == ord('s'):
#             self.msg.linear.x = -0.2
#             self.msg.angular.x = 0
#             rospy.loginfo(' s pressed')
#         elif k == ord('d'):
#             self.msg.linear.y = 0.2
#             self.msg.angular.x = 0.0
#             rospy.loginfo(' d pressed')
#         elif k == ord('a'):
#             self.msg.linear.y = -0.2
#             self.msg.angular.x = 0.0
#             rospy.loginfo(' a pressed')
#         elif k == ord('q'):
#             self.msg.linear.y = 0.0
#             self.msg.angular.z = -0.2
#             rospy.loginfo(' q pressed')
#         elif k == ord('e'):
#             self.msg.linear.y = 0.0
#             self.msg.angular.z = 0.2
#             rospy.loginfo(' e pressed')
#         elif k == ord('r'):
#             rospy.signal_shutdown('shutdown')
#             cv.destroyAllWindows()
#         else:
#             self.msg.linear.x = 0
#             self.msg.linear.y = 0
#             self.msg.angular.z = 0
#         self.pub.publish(self.msg)
#         self.rate.sleep()
#
# if __name__ == '__main__':
#     kt = KeyboardTeleop()
#     try:
#         if not rospy.is_shutdown():
#             rospy.spin()
#
#     except rospy.ROSInterruptException as e:
#         print(e)
