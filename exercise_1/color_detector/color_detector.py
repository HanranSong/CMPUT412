#!/usr/bin/env python3
import cv2
import numpy as np
from time import sleep


def gst_pipeline_string():
    # Parameters from the camera_node
    # Refer here : https://github.com/duckietown/dt-duckiebot-interface/blob/daffy/packages/camera_driver/config/jetson_nano_camera_node/duckiebot.yaml
    res_w, res_h, fps = 640, 480, 30
    fov = "full"
    # find best mode
    camera_mode = 3  #
    # compile gst pipeline
    gst_pipeline = """ \
            nvarguscamerasrc \
            sensor-mode= exposuretimerange="100000 80000000" ! \
            video/x-raw(memory:NVMM), width=, height=, format=NV12, 
                framerate=/1 ! \
            nvjpegenc ! \
            appsink \
        """.format(
        camera_mode, res_w, res_h, fps
    )

    # ---
    print("Using GST pipeline: ``".format(gst_pipeline))
    return gst_pipeline


cap = cv2.VideoCapture()
cap.open(gst_pipeline_string(), cv2.CAP_GSTREAMER)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Put here your code!
    # You can now treat output as a normal numpy array
    # Do your magic
    if not ret:
        print("Failed to capture image")
        continue

    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define blue color range in HSV
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])

    # Threshold the image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Check if blue is detected
    if cv2.countNonZero(mask) > 0:
        print("Detect Blue")

    sleep(1)
