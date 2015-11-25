import cv2, math, numpy
from operator import itemgetter

srcImage = cv2.imread("stairs.png")
if not srcImage.data: print("Failed to load image.")

depthImage = cv2.imread("depth.png")
if not depthImage.data: print("Failed to load depth image.")

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
i = 0
skipNext = False
threshold = 20
for p, q, m in coords:
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
i = 0
for p, q, m in filteredCoords:
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
i = 0
threshold = 3
for p, q in secTimeFiltered:
	if i < len(secTimeFiltered) - 1:
		# enlarge p to the left
		d     = int(depthImage[p[1]][p[0]][0])
		dp    = p[0]
		while (dp > 0):
			if math.fabs(int(depthImage[p[1]][dp][0]) - d) < threshold:
				dp -= 10
			else:
				break

		# enlarge q to the right
		d     = int(depthImage[q[1]][q[0]][0])
		dq    = q[0]
		while (dq < width):
			if math.fabs(int(depthImage[q[1]][dq][0]) - d) < threshold:
				dq += 10
			else:
				break
		secTimeFiltered[i] = ((dp, p[1]), (dq, q[1]))

		i += 1

#
# filter again lines width nearly identical height values and take averge value
#
trdTimeFiltered = []
i = 0
skipNext = False
threshold = 10
for p, q in secTimeFiltered:
	if i < len(secTimeFiltered) - 1:
		if skipNext:
			skipNext = False
		else:
			# filter lowest 20 pixels near the bottom
			if math.fabs(height - p[1]) > 20:
				trdTimeFiltered.append((p, q))
			if math.fabs(p[1] - secTimeFiltered[i+1][0][1]) < threshold:
				skipNext = True
		i += 1

# draw a line for each line
for p, q in trdTimeFiltered:
	cv2.line(srcImage, (p[0], p[1]), (q[0], q[1]), (0, 0, 255), 2)
	i += 1

stairFronts = []
i = 0
threshold = 3
for p, q in trdTimeFiltered:
	if i < len(trdTimeFiltered) - 1:
		depth = int(depthImage[p[1]][p[0]][0])
		nextCoords = (trdTimeFiltered[i+1][0][0], trdTimeFiltered[i+1][0][1])
		depthNext = int(depthImage[trdTimeFiltered[i+1][0][1]][trdTimeFiltered[i+1][0][0]][0])
		#print((depth, depthNext))
		# compare this depth to depth of the next line
		# if both have nearly the same depth value, this could be the front of a stair
		# if not check the values next to the line
		if math.fabs(depth - depthNext) < threshold:
			 stairFronts.append((trdTimeFiltered[i+1][0], q))
		else:
			nextCoords = (trdTimeFiltered[i+1][0][0], trdTimeFiltered[i+1][0][1] + 50)
			print((nextCoords, (depth, depthNext)))
			depthNext = int(depthImage[nextCoords[1]][nextCoords[0]][0])
			if math.fabs(depth - depthNext) < 15:
				stairFronts.append((trdTimeFiltered[i+1][0], q))
		i += 1

# draw a rectangle for each stair front

for p, q in stairFronts:
	cv2.rectangle(srcImage, p, q, (0, 255, 0), -1)

cv2.imwrite("out.jpg", srcImage)
