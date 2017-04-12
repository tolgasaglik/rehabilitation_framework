import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# Set up blob detector with parameters (goal is to find black concave blobs)
params = cv2.SimpleBlobDetector_Params()
#params.filterByColor = True
#params.blobColor = 10
params.filterByConvexity = True
params.minConvexity = 0.2
params.maxConvexity = 0.5
#params.filterByCircularity = True
#params.minCircularity = 0.0
#params.maxCircularity = 0.3

# define thresholds for black/gray glove color
# previous thresholds (for reference only...)
#glove_color_min = np.array([60,15,0], dtype=np.uint8)
#glove_color_max = np.array([10,170,110], dtype=np.uint8)

# current thresholds, working very good so far
glove_color_min = np.array([60,15,0], dtype=np.uint8)
glove_color_max = np.array([105,170,110], dtype=np.uint8)

ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
    detector = cv2.SimpleBlobDetector(params)
else : 
    detector = cv2.SimpleBlobDetector_create(params)

print "Press the \"q\" key to quit."

while(1):
    	# Take each frame and apply Gaussian blur to remove noise,
	# effectively turning the detected glove into a more circular shape, which is easier to detect (TODO: switch to thresholding, maybe is faster?)
    	ret,frame = cap.read()
	cv2.imshow("Original", frame)
	frame_with_blur = cv2.GaussianBlur(frame, (5,5), 1)
	frame_hsv = cv2.cvtColor(frame_with_blur, cv2.COLOR_BGR2HSV)
	# NOTE: mask is in black/white, hence the AND in the next line
	mask = cv2.inRange(frame_hsv, glove_color_min, glove_color_max)
	#frame_hsv_with_mask = cv2.bitwise_and(frame_with_blur,(0,165,255), mask=mask)
	frame_hsv_with_mask = cv2.bitwise_and(frame_with_blur,frame_with_blur, mask=mask)

	# Detect blobs
	keypoints = detector.detect(frame_hsv_with_mask)
 

	# find contours of glove and draw them on image
	frame_with_contours, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	cv2.convexHull(contours, hull)
	print hull
	cv2.drawContours(frame_with_contours, contours, -1, (255,255,0), 3)
 
	# Show keypoints
	if len(contours) > 0:
		print "Contours detected! Number of objects in image: ", str(len(contours))
	cv2.imshow("Mask", mask)
	cv2.imshow("Countours found", frame_with_contours)
	if cv2.waitKey(1) & 0xFF == ord('q'):
        	break

cap.release()
cv2.destroyAllWindows()
