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

CAM_INT_FNAME = './configs/cali_640x480.yaml'
CAM_EXT_FNAME = './configs/cali_extrinsic.yaml'
SERV_CONFIG_FNAME = './configs/server_config.yaml'

# Arugs parsing
ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=50,
                help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
                help="Whether or not frames should be displayed")
args = vars(ap.parse_args())


class MarkerDecoder:
    def __init__(self):
        """
        Loading configs
        """
        with open(SERV_CONFIG_FNAME, 'r') as f:
            self.conf_serv = yaml.safe_load(f)

        # Intrinsic parameters
        cam_int = cv2.FileStorage(CAM_INT_FNAME, cv2.FILE_STORAGE_READ)
        self.mtx = cam_int.getNode("matrix_coefficients").mat()
        self.dist = cam_int.getNode("distortion_coefficients").mat()
        self.resolution = (int(cam_int.getNode("image_width").real()), int(
            cam_int.getNode("image_height").real()))

        # Extrinsic parameters
        cam_ext = cv2.FileStorage(CAM_EXT_FNAME, cv2.FILE_STORAGE_READ)
        rvec = cam_ext.getNode("rvecs").mat()
        tvec = cam_ext.getNode("tvecs").mat()
        cRo, _ = cv2.Rodrigues(rvec)
        cTo = np.vstack((np.hstack((cRo, tvec)), [0, 0, 0, 1]))
        self.oTc = np.linalg.inv(cTo)
        print(self.oTc)

        self.__socket_connect()
        self.mode = self.conf_serv['mode']

    def signal_handler(self, signum, frame):
        print("[INFO] Leaving program...")
        # do a bit of cleanup
        self.vs.stop()
        cv2.destroyAllWindows()
        sys.exit(0)

    def __3d_extraction(self, corner, frame):
        """
        The returned transformation is the one that transforms 
        points from each marker coordinate system to the camera 
        coordinate system.
        """
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(
            corner, 0.1,
            self.mtx, self.dist)

        # get rid of that nasty numpy value array error
        (rvec - tvec).any()

        # Coordinate transformation
        cP = np.append(tvec, 1).T
        # print(cP)
        oP = self.oTc @ cP

        msg = np.squeeze(oP[:3]).tolist()

        # Draw A square around the markers
        if args["display"] > 0:
            aruco.drawDetectedMarkers(frame, corners)

            aruco.drawAxis(frame,
                           seklf.mtx,
                           self.dist,
                           rvec, tvec, 0.01)

        return msg

    def __2d_extraction(self, corners):
        msg = corners.flatten().tolist()
        return msg

    def __socket_connect(self):
        # UDP setup
        # Sending data to the server
        self.UDP_client = socket.socket(
            family=socket.AF_INET,
            type=socket.SOCK_DGRAM)
        self.UDP_client.connect(
            (self.conf_serv['serv_addr'],
             self.conf_serv['serv_port']))

    def __socket_send(self, data):
        udata = str.encode(data)
        self.UDP_client.send(udata)

    def run(self):
        self.vs = PiVideoStream(self.resolution).start()
        time.sleep(1.0)

        signal.signal(signal.SIGINT, self.signal_handler)
        pre_frame_time = time.time()
        print("[INFO] Start extracting...")

        # Aruco marker setting
        # Use 5x5 dictionary to find markers
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        # Marker detection parameters
        parameters = aruco.DetectorParameters_create()

        count = 0
        obj_list = np.zeros(10)
        while True:
            frame = self.vs.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected_img_points = aruco.detectMarkers(
                gray,
                aruco_dict,
                parameters=parameters,
                cameraMatrix=self.mtx,
                distCoeff=self.dist)

            if np.all(ids is not None):  # If there are markers found by detector
                tag_info = []
                for i in range(0, len(ids)):  # Iterate in markers
                    obj_list[int(ids[i, 0])] = 1
                    if self.mode == '3d':
                        msg = self.__3d_extraction(corners[i], frame)
                    elif self.mode == '2d':
                        msg = self.__2d_extraction(corners[i])

                    tag_info.append({'oid': int(ids[i, 0]), 'pos': msg})

                print(json.dumps(tag_info))
                self.__socket_send(json.dumps({
                    'cid': self.conf_serv['client_id'],
                    'objs': tag_info}))

            # check to see if the frame should be displayed to our screen
            if args["display"] > 0:
                frame = imutils.resize(frame, width=400)
                cv2.imshow("Frame", frame)
                key = cv2.waitKey(1) & 0xFF

            if count % 10 == 0:
                now_frame_time = time.time()
                fps = 10 / (now_frame_time - pre_frame_time)
                pre_frame_time = now_frame_time
                print("FPS: {:.2f}, obj: {}".format(
                    fps, np.array2string(obj_list)))
                obj_list = np.zeros(10)

            # update the FPS counter
            count += 1


def main():
    md = MarkerDecoder()
    md.run()


if __name__ == '__main__':
    main()
