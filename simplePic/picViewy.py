#!/usr/bin/env python
import roslib
roslib.load_manifest('simplePic')
import rospy
import tf
import cv2
import sys
import numpy as np


if __name__== '__main__':
    rospy.init_node('picViewy', log_level=rospy.DEBUG)

    cv2.namedWindow("imagewindow", 1)
    screen = np.zeros((640,480,3),np.uint8)

    listener = tf.TransformListener()
    
    rospy.logwarn("done setting up")

    rate = rospy.Rate(30)
    rateslow = rospy.Rate(1)
    
    x=0.0
    y=0.0

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('head_1', 'openni_depth_frame', rospy.Time(0))
            #rospy.loginfo("your heads at %s"%trans[0])
            x=trans[0]
            y=trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("no body")
            rateslow.sleep()
            continue
        
        cv2.circle(screen, (0,0), 20, (0,0,255), -1)

        cv2.waitKey(1)
        rate.sleep()
