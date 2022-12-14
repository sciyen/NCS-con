#!/usr/bin/env python3

import argparse
import cv2
import numpy as np
import os

# Arugs parsing
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=20,
	help="# of frames to calibrate")
ap.add_argument("-d", "--display", type=int, default=-1,
	help="Whether or not frames should be displayed")
args = vars(ap.parse_args())


# Defining the dimensions of checkerboard
gsize = 0.023
CHECKERBOARD = (6, 9)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = []


# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = gsize * np.mgrid[0:CHECKERBOARD[0],
                                  0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

# Extracting path of individual image stored in a given directory

cap = cv2.VideoCapture(0)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))  # float
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))  # float
fps = int(round(cap.get(cv2.CAP_PROP_FPS)))
frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
print((width, height))
print("FPS=", fps)

fname = "cali_{}x{}.yaml".format(width, height)

i = 0

if cap.isOpened() is not True:
    print("Failed to open camera")
else:
    while i < args['num_frames']:
        #cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        print("Recording {}/{}".format(i, args['num_frames']))

        ret, img = cap.read()

        if ret != True or img is None:
            break

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        """
        If desired number of corner are detected,
        we refine the pixel coordinates and display 
        them on the images of checker board
        """
        if ret == True:
            objpoints.append(objp)
            # refining pixel coordinates for given 2d points.
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)

            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            i += 1

        if args['display'] > 0:
            cv2.imshow('img', img)
        cv2.waitKey(10)

cap.release()
cv2.destroyAllWindows()


"""
Performing camera calibration by 
passing the value of known 3D points (objpoints)
and corresponding pixel coordinates of the 
detected corners (imgpoints)
"""

print("Calibrating...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix : \n")
print(mtx)
print("dist : \n")
print(dist)
print("rvecs : \n")
print(rvecs)
print("tvecs : \n")
print(tvecs)

f = cv2.FileStorage(fname, cv2.FileStorage_WRITE)
f.write('image_width', width)
f.write('image_height', height)
f.write('matrix_coefficients', mtx)
f.write('distortion_coefficients', dist)
f.release()
