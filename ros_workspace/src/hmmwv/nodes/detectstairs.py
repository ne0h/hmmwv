import cv2, math, numpy
from operator import itemgetter

def findStairway(srcImage, depthImage):

	# transform source image to grayscale and afterwards to binary
	grayImage   = cv2.cvtColor(srcImage, cv2.COLOR_BGR2GRAY)
	binaryImage = cv2.adaptiveThreshold(~grayImage, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, -2)

	lines = binaryImage.shape[1] / 30
	structure = cv2.getStructuringElement(cv2.MORPH_RECT, (lines, 1))

	binaryImage = cv2.erode(binaryImage, structure, (-1, -1))
	binaryImage = cv2.dilate(binaryImage, structure, (-1, -1))

	edges = cv2.Canny(binaryImage, 50, 100, apertureSize=3)
	lines = cv2.HoughLinesP(edges, 1, math.pi/180, 150, 0, 0)

	height, width = binaryImage.shape
	coords = []
	for l in lines[0]:
		coords.append(((l[0], l[1]), (l[2], l[3]), (l[1]+l[3])/2))

	#
	# sort lines from bottom to top
	#
	coords.sort(key=lambda tup: (tup[0][1]+tup[1][1])/2, reverse=True)

	#
	# filter lines width nearly identical height values and take averge values
	#
	filteredCoords = []
	skipNext = False
	threshold = 20
	for i in range(0, len(coords)):
		p, q, m = coords[i]
		if skipNext:
			skipNext = False
		else:
			# filter lowest 20 pixels
			if height - m > 20:
				if i < len(coords) - 1 and (m - coords[i+1][2]) < threshold:
					pNew = ((p[0] + coords[i+1][0][0]) / 2, (p[1] + coords[i+1][0][1]) / 2)
					qNew = ((q[0] + coords[i+1][1][0]) / 2, (q[1] + coords[i+1][1][1]) / 2)
					filteredCoords.append((pNew, qNew, m))
					skipNext = True
				else:
					filteredCoords.append((p, q, m))

		i += 1

	#
	# filter lines with same depth value
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
				secTimeFiltered.append((p, q))
				if depthImage[m][width/2][0] == depthImage[mNext][width/2][0]:
					skipNext = True
		i += 1

	#
	# try to enlarge lines in both directions. take depth values
	#
	step = 5
	threshold = 3
	for i in range(0, len(secTimeFiltered) - 1):
		p, q = secTimeFiltered[i]
		
		# enlarge p to the left
		d  = int(depthImage[p[1]][p[0]][0])
		dp = p[0]
		while (dp > 0):
			if math.fabs(int(depthImage[p[1]][dp][0]) - d) < threshold:
				dp -= 10
			else:
				break

		# enlarge q to the right
		d  = int(depthImage[q[1]][q[0]][0])
		dq = q[0]
		while (dq < width):
			if math.fabs(int(depthImage[q[1]][dq][0]) - d) < threshold:
				dq += 10
			else:
				break
		secTimeFiltered[i] = ((dp, p[1]), (dq, q[1]))

	#
	# filter again lines width nearly identical height values and take averge value
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
		cv2.line(srcImage, (p[0], p[1]), (q[0], q[1]), (0, 0, 255), 2)

	stairsFound = []
	threshold = 3
	# Compare depth values of every second line with the depth value of the next line to the bottom and the next line to
	# the top to find stairs.
	# If the depth value does not fit (e.g., due to artefacts on the depth image) check points next to this one.
	for i in range(0, len(trdTimeFiltered)):
			p, q = trdTimeFiltered[i]
			depth = int(depthImage[p[1]][p[0]][0])

			# compare values, check if the indexes exist before
			# start with value next to the top...
			if i < len(trdTimeFiltered) - 1:
				pNext, qNext = trdTimeFiltered[i+1]
				depthNext = int(depthImage[pNext[1]][qNext[0]][0])

				if math.fabs(depth - depthNext) < threshold:
					stairsFound.append((pNext, q))
				# if nothing found check value 
				else:
					newCoords = (pNext[0], pNext[1] - 30)
					depthNew = int(depthImage[newCoords[1]][newCoords[0]][0])
					if math.fabs(depth - depthNew) < 15:
						stairsFound.append((pNext, q))
			# ...and go on with prev value
			if i > 1:
				pPrev, qPrev = trdTimeFiltered[i-1]
				depthPrev = depthNext = int(depthImage[pPrev[1]][qPrev[0]][0])

				if math.fabs(depth - depthPrev) < threshold:
					stairsFound.append((p, qPrev))
				# if nothing found check value 
				else:
					newCoords = (pPrev[0], pPrev[1] + 30)
					depthNew = int(depthImage[newCoords[1]][newCoords[0]][0])
					if math.fabs(depth - depthNew) < 15:
						stairsFound.append((p, qPrev))

	return (len(stairsFound) > 1)

srcImage = cv2.imread("stairs.png")
if not srcImage.data: print("Failed to load image.")

depthImage = cv2.imread("depth.png")
if not depthImage.data: print("Failed to load depth image.")

if findStairway(srcImage, depthImage):
	print("Stairway")
else:
	print("No stairway")
