#!/usr/bin/env python

import cv2, math, numpy
from operator import itemgetter

def enlargeLine(p, q, depthImage, width, threshold=0.05, step=1):
		
	# enlarge p to the left
	d  = float(depthImage[p[1]][p[0]])
	dp = p[0]
	while (dp > 0):
		if math.fabs(float(depthImage[p[1]][dp]) - d) < threshold:
			dp -= step
		else:
			break

	# enlarge q to the right
	d  = float(depthImage[q[1]][q[0]])
	dq = q[0]
	while (dq < width):
		if math.fabs(float(depthImage[q[1]][dq]) - d) < threshold:
			dq += step
		else:
			break

	return ((dp, p[1]), (dq, q[1]))

def findStairway(rgbImage, depthImage):

	# transform source image to grayscale and afterwards to binary
	grayImage   = cv2.cvtColor(rgbImage, cv2.COLOR_BGR2GRAY)

	lines = grayImage.shape[1] / 30
	structure = cv2.getStructuringElement(cv2.MORPH_RECT, (lines, 1))

	grayImage = cv2.erode(grayImage, structure, (-1, -1))
	grayImage = cv2.dilate(grayImage, structure, (-1, -1))

	edges = cv2.Canny(grayImage, 50, 200)
	cv2.imwrite("canny.jpg", edges)
	lines = cv2.HoughLinesP(edges, 1, math.pi/180, 80, 30, 10)

	# dimensions of the input images
	height, width = grayImage.shape

	coords = []
	for l in lines[0]:
		# 				left point   right point   average height
		coords.append(((l[0], l[1]), (l[2], l[3]), (l[1]+l[3])/2))

	#
	# sort lines from bottom to top
	#
	coords.sort(key=lambda tUp: (tUp[0][1]+tUp[1][1])/2, reverse=True)

	#
	# Enlarge lowest line to the left and the right and calculate the width of the staircase.
	#
	bottomP, bottomQ = enlargeLine(coords[0][0], coords[0][1], depthImage, width)

	#
	# Remove all lines that are left or right of the base line.
	#
	leftRightFiltered = []
	for p, q, avgHgt in coords:
		if not (p[0] < bottomP[0] or q[0] > bottomQ[0]):
			leftRightFiltered.append((p, q, avgHgt))
	
	#
	# Remove lines with same depth value.
	#
	threshold = 0.05
	depthFiltered = []
	doNotAdd = []
	for i in range(0, len(leftRightFiltered)):
		pCur, qCur, avgCur = leftRightFiltered[i]
		for j in range(i + 1, len(leftRightFiltered)):
			pNew, qNew, avgNew = leftRightFiltered[j]
			if math.fabs(depthImage[avgCur][pCur[0]] - depthImage[avgNew][pNew[0]]) < threshold:
				doNotAdd.append(i)
				break
	for i in range(0, len(leftRightFiltered)):
		if i not in doNotAdd:
			depthFiltered.append((leftRightFiltered[i]))

	#
	# Try to enlarge lines in both directions. Therefor take depth values
	#
	for i in range(0, len(depthFiltered)):
		p, q = enlargeLine(depthFiltered[i][0], depthFiltered[i][1], depthImage, width)
		depthFiltered[i] = (p, q, depthFiltered[i][2])

	#
	# Filter lines width nearly identical height values.
	#
	sameHeightFiltered = []
	skipNext = False
	threshold = 10
	for i in range(0, len(depthFiltered)):
		p, q, avgHgt = depthFiltered[i]
		if skipNext:
			skipNext = False
		else:
			# filter lowest 20 pixels
			if height - avgHgt > 20:
				if i < len(depthFiltered) - 1 and (avgHgt - depthFiltered[i+1][2]) < threshold:
					pNew = ((p[0] + depthFiltered[i+1][0][0]) / 2, (p[1] + depthFiltered[i+1][0][1]) / 2)
					qNew = ((q[0] + depthFiltered[i+1][1][0]) / 2, (q[1] + depthFiltered[i+1][1][1]) / 2)
					sameHeightFiltered.append((pNew, qNew, avgHgt))
					skipNext = True
				else:
					sameHeightFiltered.append((p, q, avgHgt))

	# Compare depth values of every second line with the depth value of the next line to
	# the bottom and the next line to the top to find stairs.
	# If the depth value does not fit (e.g., due to artefacts on the depth image)
	# check points next to this one.
	stairFronts = []
	threshold = 0.1
	for i in range(1, len(sameHeightFiltered)):
			p, q, _ = sameHeightFiltered[i]
			depth = float(depthImage[p[1]][p[0]])

			# compare values, check if the indexes exist before
			# start with value next to the top...
			if i < len(sameHeightFiltered) - 1:
				pNext, qNext, _ = sameHeightFiltered[i+1]
				try:
					depthNext = float(depthImage[pNext[1]][qNext[0]])
				except:
					print("at shit")
					print(pNext)
					print(qNext)

				if math.fabs(depth - depthNext) < threshold:
					stairFronts.append((pNext, q))
				# if nothing found check value 
				else:
					newCoords = (pNext[0], pNext[1] - 30)
					depthNew = float(depthImage[newCoords[1]][newCoords[0]])
					if math.fabs(depth - depthNew) < 15:
						stairFronts.append((pNext, q))
			# ...and go on with prev value
			if i > 1:
				pPrev, qPrev, _ = sameHeightFiltered[i-1]
				depthPrev = depthNext = float(depthImage[pPrev[1]][qPrev[0]])

				if math.fabs(depth - depthPrev) < threshold:
					stairFronts.append((p, qPrev))
				# if nothing found check value 
				else:
					newCoords = (pPrev[0], pPrev[1] + 30)
					depthNew = float(depthImage[newCoords[1]][newCoords[0]])
					if math.fabs(depth - depthNew) < 15:
						stairFronts.append((p, qPrev))

	"""
	for i in range(0, len(sameHeightFiltered)):
		p, q, _ = sameHeightFiltered[i]
		cv2.line(rgbImage, (p[0], p[1]), (q[0], q[1]), (0, 0, 255), 2)
	cv2.imwrite("rgb.jpg", rgbImage)

	for p, q in stairFronts:
		cv2.rectangle(rgbImage, p, q, (0, 255, 0))
	cv2.imwrite("rgb.jpg", rgbImage)
	"""

	return (len(stairFronts) > 1)
