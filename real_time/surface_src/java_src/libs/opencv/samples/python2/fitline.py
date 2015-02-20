#!/usr/bin/env python

'''
Robust line fitting.
==================

Example of using cv2.fitLine function for fitting line
to points in presence of outliers.

Usage
-----
fitline.py

Switch through different M-estimator functions and see,
how well the robust functions fit the line even
in case of ~50% of outliers.

Keys
----
SPACE - generaty random points
f     - change distance function
ESC   - exit
'''

import numpy as np
import cv2
import itertools as it
from common import draw_str


w, h = 512, 256

def toint(p):
    return tuple(map(int, p))

def sample_line(p1, p2, n, noise=0.0):
    p1 = np.float32(p1)
    t = np.random.rand(n,1)
    return p1 + (p2-p1)*t + np.random.normal(size=(n, 2))*noise

dist_func_names = it.cycle('CV_DIST_L2 CV_DIST_L1 CV_DIST_L12 CV_DIST_FAIR CV_DIST_WELSCH CV_DIST_HUBER'.split())
cur_func_name = dist_func_names.next()

def update(_=None):
    noise = cv2.getTrackbarPos('noise', 'fit line')
    n = cv2.getTrackbarPos('point n', 'fit line')
    r = cv2.getTrackbarPos('outlier %', 'fit line') / 100.0
    outn = int(n*r)

    p0, p1 = (90, 80), (w-90, h-80)
    img = np.zeros((h, w, 3), np.uint8)
    cv2.line(img, toint(p0), toint(p1), (0, 255, 0))

    if n > 0:
        line_points = sample_line(p0, p1, n-outn, noise)
        outliers = np.random.rand(outn, 2) * (w, h)
        points = np.vstack([line_points, outliers])
        for p in line_points:
            cv2.circle(img, toint(p), 2, (255, 255, 255), -1)
        for p in outliers:
            cv2.circle(img, toint(p), 2, (64, 64, 255), -1)
        func = getattr(cv2.cv, cur_func_name)
        vx, vy, cx, cy = cv2.fitLine(np.float32(points), func, 0, 0.01, 0.01)
        cv2.line(img, (int(cx-vx*w), int(cy-vy*w)), (int(cx+vx*w), int(cy+vy*w)), (0, 0, 255))

    draw_str(img, (20, 20), cur_func_name)
    cv2.imshow('fit line', img)

if __name__ == '__main__':
    print __doc__

    cv2.namedWindow('fit line')
    cv2.createTrackbar('noise', 'fit line', 3, 50, update)
    cv2.createTrackbar('point n', 'fit line', 100, 500, update)
    cv2.createTrackbar('outlier %', 'fit line', 30, 100, update)
    while True:
        update()
        ch = cv2.waitKey(0)
        if ch == ord('f'):
            cur_func_name = dist_func_names.next()
        if ch == 27:
            break
