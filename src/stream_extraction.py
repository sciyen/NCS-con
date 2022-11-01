# import the necessary packages
from __future__ import print_function
from imutils.video.pivideostream import PiVideoStream
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import signal
import sys

import numpy as np
import socket
import json
import yaml

import cv2
import cv2.aruco as aruco

print("[INFO] Program warming up")

CAM_CONFIG_FNAME = './configs/cali_640x480.yaml'
SERV_CONFIG_FNAME = './configs/server_config.yaml'

# Arugs parsing
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=50,
	help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
	help="Whether or not frames should be displayed")
args = vars(ap.parse_args())

# Loading configs
with open(SERV_CONFIG_FNAME, 'r') as f:
    conf_serv = yaml.safe_load(f)

conf_cam = cv2.FileStorage(CAM_CONFIG_FNAME, cv2.FILE_STORAGE_READ)
mat_coef = conf_cam.getNode("matrix_coefficients").mat()
dist_coef = conf_cam.getNode("distortion_coefficients").mat()
resolution = (int(conf_cam.getNode("image_width").real()), int(conf_cam.getNode("image_height").real()))
 
# UDP setup
UDP_client = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDP_client.connect((conf_serv['serv_addr'], conf_serv['serv_port']))
 
def main():
   
    # Aruco marker setting
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
    parameters = aruco.DetectorParameters_create()  # Marker detection parameters

    vs = PiVideoStream(resolution).start()
    time.sleep(1.0)

    def signal_handler(signum, frame):
        print("[INFO] Leaving program...")
        cv2.destroyAllWindows()
        vs.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    pre_frame_time = time.time()
    print("[INFO] Start extracting...")

    count = 0
    obj_list = np.zeros(10)
    while True:
        frame = vs.read()
    
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
        corners, ids, rejected_img_points = aruco.detectMarkers(
                gray, 
                aruco_dict, 
                parameters=parameters, 
                cameraMatrix=mat_coef,
                distCoeff=dist_coef)
    
        if np.all(ids is not None):  # If there are markers found by detector
            tag_info = []
            for i in range(0, len(ids)):  # Iterate in markers
                obj_list[int(ids[i, 0])] = 1
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 
                        0.1, 
                        mat_coef, 
                        dist_coef)
                
                (rvec - tvec).any()  # get rid of that nasty numpy value array error
                aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
    
                aruco.drawAxis(frame, 
                        mat_coef, 
                        dist_coef, 
                        rvec, tvec, 0.01)  # Draw Axis
                tag_info.append({'oid': int(ids[i, 0]), 'pos': np.squeeze(tvec).tolist()})

            socket_send(json.dumps({'cid': conf_serv['client_id'], 'objs': tag_info}))
    
        # check to see if the frame should be displayed to our screen
        if args["display"] > 0:
            frame = imutils.resize(frame, width=400)
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF

        if count % 10 == 0:
            now_frame_time = time.time()
            fps = 10 / (now_frame_time - pre_frame_time)
            pre_frame_time = now_frame_time
            print("FPS: {:.2f}, obj: {}".format(fps, np.array2string(obj_list)))
            obj_list = np.zeros(10)

        # update the FPS counter
        count += 1

    # do a bit of cleanup
    cv2.destroyAllWindows()
    vs.stop()

"""
Sending data to the server
"""
def socket_send(data):
    udata = str.encode(data)
    UDP_client.send(udata)

if __name__ == '__main__':
    main()

