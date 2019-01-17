import imutils
import cv2 as cv

image = cv.imread("example.png")  # will be frame from footage
cv.imshow("Image", image)
(h, w, d) = image.shape  # d for depth (num color channels)

# for example take screenshot of stop sign footage

startY = 0
startX = 0
endY = 0
endX = 0

# initial stop sign tracking
stop_init = image[startY:endY, startX:endX]

# what is ret
def frame_to_video(ret, frame, track_window):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    dst = cv.calcBackProject([hsv], [0], roi_hist, [0, 180], 1)

    # run each frame through meanshift
    # apply meanshift to get the new location
    ret, track_window = cv.meanShift(dst, track_window, term_crit)

    # export frame to new video
    cv.imwrite("result.jpg", img2)
