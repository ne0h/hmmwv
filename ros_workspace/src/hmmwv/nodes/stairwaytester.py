#!/usr/bin/env python

import cv2, numpy
import detectstairs

rgbImage = cv2.imread("stairs.png")
depthImage = cv2.imread("depth.png")
height, width, _ = depthImage.shape

depthImage2 = numpy.zeros(shape=(height, width))
for i in range(0, height):
	for j in range(0, width):
		depthImage2[i][j] = int(depthImage[i][j][0])

if detectstairs.findStairway(rgbImage, depthImage2):
	print("Stairway!")
else:
	print("No stairway.")
