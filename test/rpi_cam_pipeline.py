import sys
sys.path.append("/home/wyc/third_party_libraries/apriltag/build/")

from apriltag import apriltag
from apriltag_util import *

from pprint import pprint

import cv2
import numpy as np

from picamera2 import Picamera2, Preview
from libcamera import controls
import time


picam2 = Picamera2()
# mid resolution, full sensor scene
# raw resolution: 4608, 2592
# fps: 56.03
# format: SRGGB10_CSI2P
mode = picam2.sensor_modes[1]

# low resolution output: 648x648
# config = picam2.create_still_configuration(lores={"size": (648, 648), "format": "RGB888"}, sensor={'output_size': mode['size']}, encode="lores", buffer_count=3, queue=False)

config = picam2.create_still_configuration(lores={"size": (648, 648), "format": "RGB888"}, sensor={'output_size': mode['size']}, encode="lores", buffer_count=2, queue=False)
picam2.configure(config)
time.sleep(.5)

picam2.start()

# Set crop area: center 1296x1296
picam2.set_controls( {
    # "ScalerCrop": (504*2, 0, 1296*2, 1296*2),
    "ScalerCrop": (1104, 96, 2400, 2400),
    # "AeEnable": False,
    "ExposureTime": 2000,
    # "AnalogueGain": 4.0,
    # "AfMode": controls.AfModeEnum.Manual,
    # "LensPosition": 1.0,
    # "Brightness": 0.2,
    "Contrast": 1.2,
})


print(picam2.capture_metadata()["ExposureTime"])

timer = Timer()
detector = apriltag("tagStandard41h12")


rate = 1/56.03

no_ball_count = 0

test_time = 0.2
run_count = int(test_time/rate)

process_timer = Timer()
total_timer = Timer()

counter = 0
while True:
    if (process_timer.lap() < rate):
        continue
    if (counter > run_count):
        break
    counter = counter + 1
    
    process_timer.reset()
    timer.reset()
    # frame = picam2.capture_array("lores")
    # req = picam2.capture_request(flush=True)
    # frame = req.make_array("lores")
    # req.release()
    frame = picam2.capture_array("lores")
    print("Capture time: ", timer.lap())

    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    timer.reset()
    detections = detector.detect(frame)
    print("Tag detect time: ", timer.lap())

    timer.reset()
    circles = cv2.HoughCircles(
        frame,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=100,
        param1=200,
        param2=10,
        minRadius=12,
        maxRadius=15,
    )
    print(circles)
    print("Ball detect time: ", timer.lap())

    if circles is not None:
        print(circles[0][0])
    else:
        no_ball_count = no_ball_count + 1


print("Total time: ", total_timer.lap())

lores_label = draw_apriltag_boxes(frame, detections)
if circles is not None:
    for i in range(circles.shape[1]):
        circle = np.uint16(np.around(circles[0][i]))
        x, y, r = circle
        cv2.circle(lores_label, (x, y), r, (0, 255, 0), 2)  # Circle outline
        cv2.circle(lores_label, (x, y), 2, (0, 0, 255), 3)  # Center point


cv2.imwrite("test.jpg", frame)
cv2.imwrite("test_result.jpg", lores_label)


if no_ball_count != 0:
    print("Ball lost tracking: ", no_ball_count, "/", run_count)

# pprint(detections)

ball_position = np.array(circles[0][0][0:2])



Gpoint = [None]*4
for tag in detections:
    if tag['id'] == 7:
        G1 = tag['center']
    if tag['id'] == 19:
        G2 = tag['center']
    if tag['id'] == 23:
        G3 = tag['center']
    if tag['id'] == 37:
        G4 = tag['center']

d = 50

P = ball_position


G0 = (G1 + G2 + G3 + G4) / 4

Gpoint = [G0, G1, G2, G3, G4]


vec_G2_G1 = G1 - G2
vec_G0_P = P - G0
vec_G4_G1 = G1 - G4

scale = 2*d / np.linalg.norm(vec_G2_G1)
print("Scale: ", scale)

ball_x = np.dot( vec_G2_G1 / np.linalg.norm(vec_G2_G1) , vec_G0_P ) * scale
ball_y = np.dot( vec_G4_G1 / np.linalg.norm(vec_G4_G1) , vec_G0_P ) * scale

ball = np.array([ball_x, ball_y])

print("P: ", P)
print("G0: ", G0)
print("G1: ", G1)
print("G2: ", G2)
print("G3: ", G3)
print("G4: ", G4)
print("Ball: ", ball)

print("x axis: ", vec_G2_G1)
print("y axis: ", vec_G4_G1)
print("point vec: ", vec_G0_P)
