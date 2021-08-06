from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2

OPENCV_OBJECT_TRACKERS = {
	"csrt": cv2.TrackerCSRT_create,
	"kcf": cv2.TrackerKCF_create,
	"boosting": cv2.TrackerBoosting_create,
	"mil": cv2.TrackerMIL_create,
	"tld": cv2.TrackerTLD_create,
	"medianflow": cv2.TrackerMedianFlow_create,
	"mosse": cv2.TrackerMOSSE_create
}

trackers = cv2.MultiTracker_create()

path = r"X:\DDP\Codes\DDP-UAV-Computer-Vision\Hover 2.mkv"
vs = cv2.VideoCapture(path)

while True:
	# grab the current frame, then handle if we are using a
	# VideoStream or VideoCapture object
	frame = vs.read()
	frame = frame[1]
	# check to see if we have reached the end of the stream
	if frame is None:
		break
	# resize the frame (so we can process it faster)
	frame = imutils.resize(frame, width=600)

	# grab the updated bounding box coordinates (if any) for each
	# object that is being tracked
	(success, boxes) = trackers.update(frame)
	# loop over the bounding boxes and draw then on the frame
	for box in boxes:
		(x, y, w, h) = [int(v) for v in box]
		cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
	# show the output frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	# if the 's' key is selected, we are going to "select" a bounding
	# box to track
	if key == ord("s"):
		# select the bounding box of the object we want to track (make
		# sure you press ENTER or SPACE after selecting the ROI)
		box = cv2.selectROI("Frame", frame, fromCenter = False,
			showCrosshair = True)
		# create a new object tracker for the bounding box and add it
		# to our multi-object tracker
		tracker = OPENCV_OBJECT_TRACKERS["medianflow"]()
		trackers.add(tracker, frame, box)
	# if the `q` key was pressed, break from the loop
	elif key == ord("q"):
		break

vs.release()
# close all windows
cv2.destroyAllWindows()