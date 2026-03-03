import sys
sys.path.append("/home/wyc/third_party_libraries/apriltag/build/")
import cv2
import numpy as np
import time
from pprint import pprint
from apriltag import apriltag
from apriltag_util import *

imagepath = 'test.jpg'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
detector = apriltag("tagStandard41h12")

start_time = time.time()
detections = detector.detect(image)
print(time.time() - start_time)

vis = draw_apriltag_boxes(image, detections)

cv2.imwrite("test_result.jpg", vis)
