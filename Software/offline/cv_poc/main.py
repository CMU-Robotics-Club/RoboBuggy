import numpy as np

import imutils
import cv2 as cv

cap = cv.VideoCapture("example.flv")
# read return type None?
ret, frame = cap.read()

image = cv.imread("example.png")  # will be frame from footage
cv.imshow("Image", image)
(h, w, d) = image.shape  # d for depth (num color channels)

# for example take screenshot of stop sign footage

startY = 0
startX = 0
width = 0
height = 0

endY = startY + height
endX = startX + width
track_window = (startX, startY, width, height)

# ROI setup (region of interest)

# initial stop sign tracking
# stop_init = image[startY:endY, startX:endX]
roi = frame[startY:startY+height, startX:startX+width]
hsv_roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
mask = cv.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
roi_hist = cv.calcHist([hsv_roi],[0],mask,[180],[0,180])
cv.normalize(roi_hist,roi_hist,0,255,cv.NORM_MINMAX)


# Setup the termination criteria, either 10 iteration or move by atleast 1 pt
term_crit = ( cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 1 )


while True:
    ret, frame = cap.read()

    # what is ret
    if ret == True:  # is it actually redundant? could ret be two types
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        dst = cv.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)

        # apply meanshift
        ret, track_window = cv.CamShift(dst, track_window, term_crit)

        # draw
        pts = cv.boxPoints(ret)  # ret must be box type
        # how can ret because new coords and boolean? see if statement
        pts = np.int0(pts)  # cast points ?
        img2 = cv.polylines(frame, [pts], True, 255, 2)  # actually draw
        cv.imshow('img2', img2)

        # exit key 60? or 27 code and 60 seconds
        k = cv.waitKey(60) & 0xff
        if k == 27:
            break
        else:
            # name the new image whatever key was pressed?
            # does it only export very last image
            cv.imwrite(chr(k)+".jpg", img2)

cv.destroyAllWindows()
cap.release()


def frame_to_video(ret, frame, track_window):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    dst = cv.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)

    # run each frame through meanshift
    # apply meanshift to get the new location
    ret, track_window = cv.meanShift(dst, track_window, term_crit)

    # export frame to new video
    cv.imwrite("result.jpg", img2)
