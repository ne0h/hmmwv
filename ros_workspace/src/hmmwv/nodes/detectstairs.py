#!/usr/bin/env python

import cv2, math, numpy
from operator import itemgetter

class DetectStairs:

	def __enlargeLine(self, p, q, threshold=0.05, step=1):
		
		# enlarge p to the left
		d  = self.__getDepthValue(p[0], p[1])
		dp = p[0]
		while (dp > 0):
			if math.fabs(self.__getDepthValue(dp, p[1]) - d) < threshold:
				dp -= step
			else:
				break
		
		# enlarge q to the right
		d  = self.__getDepthValue(q[0], q[1])
		dq = q[0]
		while (dq < self.__width):
			if math.fabs(self.__getDepthValue(dq, q[1]) - d) < threshold:
				dq += step
			else:
				break
		
		return ((dp, p[1]), (dq, q[1]))

	def __removeNonHorizontalLines(self, lines, tolerance=5):
		result = []
		for p, q, avg in lines:
			if math.fabs(p[1] - q[1]) < tolerance:
				result.append((p, q, avg))
		return result

	def __removeShortLines(self, lines, minPercentage=0.1):
		result = []
		for p, q, avg in lines:
			if q[0] - p[0] > minPercentage * self.__width:
				result.append((p, q, avg))
		return result

	def __removeLinesWithSameDepth(self, lines, threshold=0.05):
		result = []
		doNotAdd = []
		for i in range(0, len(lines)):
			pCur, qCur, avgCur = lines[i]
			for j in range(i + 1, len(lines)):
				pNew, qNew, avgNew = lines[j]
				if math.fabs(self.__getDepthValue(pCur[0], avgCur[1]) - self.__getDepthValue(pNew[0], avgNew[1])) < threshold:
					doNotAdd.append(i)
					break
		for i in range(0, len(lines)):
			if i not in doNotAdd:
				result.append((lines[i]))
		return result

	def __removeLinesWithSameHeight(self, lines, threshold=10):
		result = []
		"""
		for i in range(0, len(lines)):
			pCur, qCur, avgCur = lines[i]
			for j in range(i, len(lines)):
				pNew, qNew, avgNew = lines[j]
				if math.fabs(avgCur[1] - avgNew[1]) < threshold:
					doNotAdd.append(i)
		for i in range(0, len(lines)):
			if i not in doNotAdd:
				result.append((lines[i]))
		"""

		i = 0
		while i < len(lines):
			result.append((lines[i]))
			pCur, qCur, avgCur = lines[i]
			for j in range(i, len(lines)):
				pNew, qNew, avgNew = lines[j]
				if not math.fabs(avgCur[1] - avgNew[1]) < threshold:
					i = j

		return result

	def calculate(self):

		# transform source image to grayscale and afterwards to binary
		grayImage   = cv2.cvtColor(self.__rgbImage, cv2.COLOR_BGR2GRAY)

		lines = grayImage.shape[1] / 30
		structure = cv2.getStructuringElement(cv2.MORPH_RECT, (lines, 1))

		grayImage = cv2.erode(grayImage, structure, (-1, -1))
		grayImage = cv2.dilate(grayImage, structure, (-1, -1))

		edges = cv2.Canny(grayImage, 50, 200)
		cv2.imwrite("canny.jpg", edges)
		lines = cv2.HoughLinesP(edges, 1, math.pi/180, 80, 30, 10)

		coords = []
		for l in lines[0]:
			# 				left point   right point   average width and height of both points
			coords.append(((l[0], l[1]), (l[2], l[3]), (int((l[0]+l[2])/2), int(l[1]+l[3])/2)))

		#
		# sort lines from bottom to top
		#
		coords.sort(key=lambda tUp: (tUp[0][1]+tUp[1][1])/2, reverse=True)

		#
		# Enlarge lowest line to the left and the right and calculate the width of the staircase.
		#
		bottomP, bottomQ = self.__enlargeLine(coords[0][0], coords[0][1])
		cv2.line(self.__rgbImage, (bottomP[0], bottomP[1]), (bottomQ[0], bottomQ[1]), (0, 255, 0), 2)
		#
		# Remove all lines that are left or right of the base line.
		#
		leftRightFiltered = []
		for p, q, avgHgt in coords:
			if not (p[0] < bottomP[0] or q[0] > bottomQ[0]):
				leftRightFiltered.append((p, q, avgHgt))

		#
		# Remove non-horizontal lines
		#
		horizontalFiltered = self.__removeNonHorizontalLines(leftRightFiltered)
		
		#
		# Remove lines with same depth value.
		#
		depthFiltered = self.__removeLinesWithSameDepth(horizontalFiltered)

		#
		# Try to enlarge lines in both directions. Therefor take depth values
		#
		for i in range(0, len(depthFiltered)):
			p, q = self.__enlargeLine(depthFiltered[i][0], depthFiltered[i][1])
			depthFiltered[i] = (p, q, depthFiltered[i][2])

		#
		# Remove alle lines that are shorter than 25% of the image width
		#
		shortLinesFiltered = self.__removeShortLines(depthFiltered)

		#
		# Filter lines width nearly identical height values.
		#
		heightFiltered = self.__removeLinesWithSameHeight(shortLinesFiltered)

		for i in range(0, len(heightFiltered)):
			p, q, _ = heightFiltered[i]
			cv2.line(self.__rgbImage, (p[0], p[1]), (q[0], q[1]), (0, 0, 255), 2)

		# Compare depth values of every second line with the depth value of the next line to
		# the bottom and the next line to the top to find stairs.
		# If the depth value does not fit (e.g., due to artefacts on the depth image)
		# check points next to this one.
		stairFronts = []
		threshold = 0.1
		tolerance = 10
		for i in range(1, len(heightFiltered)):
			pCur, qCur, avgCur = heightFiltered[i]
			depthCur = self.__getDepthValue(avgCur[0], avgCur[1])
			"""
			# compare values, check if the indexes exist before
			# start with value next to the top...
			if i < len(heightFiltered) - 1:
				pNext, qNext, avgNext = heightFiltered[i+1]
				depthNext = self.__getDepthValue(qNext[0], pNext[1])

				if math.fabs(depthCur - depthNext) < threshold:
					stairFronts.append((pNext, qCur))
				# if nothing found check value 
				else:
					avgCurNew = (avgCur[0], avgCur[1] - tolerance)
					depthCurNew = self.__getDepthValue(avgCurNew[0], avgCurNew[1])
					if math.fabs(depthNext - depthCurNew) < tolerance:
						stairFronts.append((pNext, q))
			"""
			# ...and go on with prev value
			if i > 0:
				pPrev, qPrev, avgPrev = heightFiltered[i-1]
				depthPrev = self.__getDepthValue(avgPrev[0], avgPrev[1])

				if math.fabs(depthCur - depthPrev) < threshold:
					stairFronts.append((pCur, qPrev))
				# if nothing found check value 
				else:
					avgCurNew = (avgCur[0], avgCur[1] + tolerance)
					depthCurNew = self.__getDepthValue(avgCurNew[0], avgCurNew[1])
					if math.fabs(depthPrev - depthCurNew) < threshold:
						stairFronts.append((pCur, qPrev))

		#for p, q in stairFronts:
		#	cv2.rectangle(rgbImage, p, q, (0, 255, 0))
		cv2.imwrite("rgb.jpg", self.__rgbImage)

		return (len(stairFronts) > 1)

	def __getDepthValue(self, width, height):
		return float(self.__depthImage[height][width])

	def __init__(self, rgbImage, depthImage):
		self.__rgbImage = rgbImage
		self.__depthImage = depthImage

		# dimensions of the input images
		self.__height, self.__width, _ = rgbImage.shape
