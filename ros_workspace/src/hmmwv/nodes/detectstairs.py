#!/usr/bin/env python

import cv2, math, numpy
from operator import itemgetter

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
	# filter lines width nearly identical height values and take averge values
	#
	filteredCoords = []
	skipNext = False
	threshold = 20
	for i in range(0, len(coords)):
		p, q, avgHgt = coords[i]
		if skipNext:
			skipNext = False
		else:
			# filter lowest 20 pixels
			if height - avgHgt > 20:
				if i < len(coords) - 1 and (avgHgt - coords[i+1][2]) < threshold:
					pNew = ((p[0] + coords[i+1][0][0]) / 2, (p[1] + coords[i+1][0][1]) / 2)
					qNew = ((q[0] + coords[i+1][1][0]) / 2, (q[1] + coords[i+1][1][1]) / 2)
					filteredCoords.append((pNew, qNew, avgHgt))
					skipNext = True
				else:
					filteredCoords.append((p, q, avgHgt))

	#
	# Remove lines with same depth value.
	#
	secTimeFiltered = []
	skipNext = False
	for i in range(0, len(filteredCoords)):
		p, q, m = filteredCoords[i]
		if skipNext:
			skipNext = False
		else:
			if i < len(filteredCoords) - 1:
				mNext = filteredCoords[i+1][2]

				# add this line and check if the next line has the same depth value.
				# If so skip the next one
				secTimeFiltered.append((p, q))

				if depthImage[m][width/2] == depthImage[mNext][width/2]:
					skipNext = True

	#
	# Try to enlarge lines in both directions. Take depth values
	#
	step = 1
	threshold = 0.05
	for i in range(0, len(secTimeFiltered) - 1):
		p, q = secTimeFiltered[i]
		
		# enlarge p to the left
		d  = float(depthImage[p[1]][p[0]])
		dp = p[0]
		while (dp > 0):
			if math.fabs(float(depthImage[p[1]][dp]) - d) < threshold:
				dp -= 10
			else:
				break

		# enlarge q to the right
		d  = float(depthImage[q[1]][q[0]])
		dq = q[0]
		while (dq < width):
			if math.fabs(float(depthImage[q[1]][dq]) - d) < threshold:
				dq += 10
			else:
				break
		secTimeFiltered[i] = ((dp, p[1]), (dq, q[1]))

	for i in range(0, len(secTimeFiltered)):
		p, q = secTimeFiltered[i]
		cv2.line(rgbImage, (p[0], p[1]), (q[0], q[1]), (0, 0, 255), 2)
	cv2.imwrite("rgb.jpg", rgbImage)

	#
	# Filter again lines width nearly identical height values and take averge value.
	#
	trdTimeFiltered = []
	skipNext = False
	threshold = 10
	for i in range(0, len(secTimeFiltered) - 1):
		p, q = secTimeFiltered[i]

		if skipNext:
			skipNext = False
		else:
			# filter lowest 20 pixels near the bottom
			if math.fabs(height - p[1]) > 20:
				trdTimeFiltered.append((p, q))
			if math.fabs(p[1] - secTimeFiltered[i+1][0][1]) < threshold:
				skipNext = True

	# draw a line for each line
	for i in range(0, len(trdTimeFiltered)):
		p, q = trdTimeFiltered[i]
		cv2.line(rgbImage, (p[0], p[1]), (q[0], q[1]), (0, 0, 255), 2)

	# Compare depth values of every second line with the depth value of the next line to
	# the bottom and the next line to the top to find stairs.
	# If the depth value does not fit (e.g., due to artefacts on the depth image)
	# check points next to this one.
	stairFronts = []
	threshold = 0.1
	for i in range(0, len(trdTimeFiltered)):
			p, q = trdTimeFiltered[i]
			depth = float(depthImage[p[1]][p[0]])

			# compare values, check if the indexes exist before
			# start with value next to the top...
			if i < len(trdTimeFiltered) - 1:
				pNext, qNext = trdTimeFiltered[i+1]
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
				pPrev, qPrev = trdTimeFiltered[i-1]
				depthPrev = depthNext = float(depthImage[pPrev[1]][qPrev[0]])

				if math.fabs(depth - depthPrev) < threshold:
					stairFronts.append((p, qPrev))
				# if nothing found check value 
				else:
					newCoords = (pPrev[0], pPrev[1] + 30)
					depthNew = float(depthImage[newCoords[1]][newCoords[0]])
					if math.fabs(depth - depthNew) < 15:
						stairFronts.append((p, qPrev))

	# Removes

	for p, q in stairFronts:
		cv2.rectangle(rgbImage, p, q, (0, 255, 0))
	#cv2.imwrite("rgb.jpg", rgbImage)

	return (len(stairFronts) > 1)
