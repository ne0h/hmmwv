#!/usr/bin/env python

import rospy, message_filters
from sensor_msgs.msg import Image

import cv2, math, numpy
from operator import itemgetter
from cv_bridge import CvBridge, CvBridgeError

class DetectStairs:

	def __getRandomColor(self):
		return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

	def __getMiddleDepthValue(self, height):
		return self.__getDepthValue(self.__width / 2, height)

	def __getDepthValue(self, width, height):
		return float(self.__depthImage[height][width])

	def __splitIntoDepths(self, threshold=0.02):
		result = []
		last = 0
		lastDepth = self.__getMiddleDepthValue(0)
		i = 1
		while i < self.__height - 1:
			if not math.fabs(lastDepth - self.__getMiddleDepthValue(i)) < threshold:
				result.append((last, i))
				last = i
				lastDepth = self.__getMiddleDepthValue(i + 1)
			i = i + 1
		return result

	def calculate(self):

		depths = self.__splitIntoDepths()
		print(len(depths))
		for d in depths:
			print("% to %", (d[0], d[1]))



		return True

	def __init__(self, rgbImage, depthImage):
		self.__rgbImage = rgbImage
		self.__depthImage = depthImage
		self.__height, self.__width, _ = rgbImage.shape

# Needs to convert ros images to opencv images (numpy array)
def callback(rgbImage, depthImage):
	print("Callback!")
	if DetectStairs(CvBridge().imgmsg_to_cv2(rgbImage, "bgr8"),	CvBridge().imgmsg_to_cv2(depthImage)).calculate():
		rospy.loginfo("Stairway!")
	else:
		rospy.loginfo("No stairway.")
	import sys
	sys.exit()

if __name__ == "__main__":
	rospy.init_node("stairsdetection")
	
	rgbImgSub   = message_filters.Subscriber("/camera/rgb/image_color", Image)
	depthImgSub = message_filters.Subscriber("/camera/depth/image", Image)
	
	ts = message_filters.ApproximateTimeSynchronizer([rgbImgSub, depthImgSub], 10, 2)
	ts.registerCallback(callback)

	rospy.spin()