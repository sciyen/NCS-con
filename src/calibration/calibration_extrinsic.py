#!/usr/bin/env python3

import argparse
import cv2
import cv2.aruco as aruco
import numpy as np

# Arugs parsing
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=20,
	help="# of frames to calibrate")
ap.add_argument("-d", "--display", type=int, default=-1,
	help="Whether or not frames should be displayed")
ap.add_argument("-i", "--input-path", type=str, default='cali_640x480.yaml.yaml',
	help="Specify output file path")
ap.add_argument("-o", "--output-path", type=str, default='cali_extrinsic.yaml',
	help="Specify output file path")
ap.add_argument("-m", "--marker", type=str, default='aruco',
	help="Specify using which kind of marker to mark origin")
args = vars(ap.parse_args())

# Aruco marker setting
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
parameters = aruco.DetectorParameters_create()  # Marker detection parameters


gsize = 0.023
CHECKERBOARD = (6, 9)
ext_fname = args['output_path']
int_fname = args['input_path']
conf_cam = cv2.FileStorage(int_fname, cv2.FILE_STORAGE_READ)

mtx = conf_cam.getNode("matrix_coefficients").mat()
dist = conf_cam.getNode("distortion_coefficients").mat()
width = int(conf_cam.getNode("image_width").real())
height = int(conf_cam.getNode("image_height").real())

# Defining the world coordinates for 3D points
if args['marker'] == 'chessboard':
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = gsize * np.mgrid[0:CHECKERBOARD[0],
                                      0:CHECKERBOARD[1]].T.reshape(-1, 2)

cap = cv2.VideoCapture(0)
rvecs = np.zeros((3, args['num_frames']))
tvecs = np.zeros((3, args['num_frames']))
i = 0

if cap.isOpened() is not True:
    print("Failed to open camera")
else:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    while i < args['num_frames']:
        print("Recording {}/{}".format(i, args['num_frames']))
        ret, img = cap.read()

        if ret != True or img is None:
            break

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if args['marker'] == 'chessboard':
            ret, corners = cv2.findChessboardCorners(
                gray, CHECKERBOARD, 
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        elif args['marker'] == 'aruco':
            corners, ids, rejected_img_points = aruco.detectMarkers(
                gray, 
                aruco_dict, 
                parameters=parameters, 
                cameraMatrix=mtx,
                distCoeff=dist)

        if ret is not True: 
            continue

        if args['marker'] == 'chessboard':
            # This function returns the rotation and the translation vectors that transform a 3D point expressed in the object coordinate frame to the camera coordinate frame
            ret, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
        elif args['marker'] == 'aruco':
            for j in range(0, len(ids)):  # Iterate in markers
                if ids[j, 0] == 0:
                    rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[j], 
                        0.1, 
                        mtx, 
                        dist)
                    rvec = np.squeeze(rvec)
                    tvec = np.squeeze(tvec)
                    ret = True

        if ret is not True: 
            continue
        
        rvecs[:, i] = rvec.T
        tvecs[:, i] = tvec.T
        cv2.waitKey(10)
        i += 1

cap.release()
cv2.destroyAllWindows()

rvecs = np.mean(rvecs, axis=1)
tvecs = np.mean(tvecs, axis=1)

"""
Saving extrinsic parameters
"""
f = cv2.FileStorage(ext_fname, cv2.FileStorage_WRITE)
f.write('rvecs', rvecs)
f.write('tvecs', tvecs)
f.release()
