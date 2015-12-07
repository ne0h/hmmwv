#!/usr/bin/env python

import rospy, message_filters
from sensor_msgs.msg import Image

import cv2, math, numpy
from operator import itemgetter
from cv_bridge import CvBridge, CvBridgeError

import detectstairs

# Needs to convert ros images to opencv images (numpy array)
def callback(rgbImage, depthImage):

	if detectstairs.findStairway(CvBridge().imgmsg_to_cv2(rgbImage, "bgr8"), CvBridge().imgmsg_to_cv2(depthImage)):
		rospy.loginfo("Stairway!")
	else:
		rospy.loginfo("No stairway.")
	import sys
	sys.exit()

if __name__ == "__main__":
	rospy.init_node("stairsdetection")
	
	rgbImgSub   = message_filters.Subscriber("/camera/rgb/image_color", Image)
	depthImgSub = message_filters.Subscriber("/camera/depth/image", Image)
	
	ts = message_filters.ApproximateTimeSynchronizer([rgbImgSub, depthImgSub], 10, 1)
	ts.registerCallback(callback)

	rospy.spin()