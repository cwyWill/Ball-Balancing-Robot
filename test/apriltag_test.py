from pprint import *
import cv2
import numpy as np
# from apriltag import apriltag
from dt_apriltags import Detector
import time

imagepath = 'test.jpg'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)

# at_detector = Detector(searchpath=['apriltags'],
#                         families='tagStandard41h12',
#                         quad_decimate=1.0,
#                         quad_sigma=0.0,
#                         refine_edges=1,
#                         decode_sharpening=0.1,
#                         debug=0)
at_detector = Detector(searchpath=['apriltags'],
                        families='tagStandard41h12')

start_time = time.time()
# tags = at_detector.detect(image, estimate_tag_pose=False, camera_params=None, tag_size=None)
tags = at_detector.detect(image)
print("Duration", time.time()-start_time)

pprint(tags[0])
pprint(len(tags))