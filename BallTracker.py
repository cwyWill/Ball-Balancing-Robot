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

class BallTracker:
    def __init__(self):
        self.picam2 = Picamera2()
        self.setCamera()
        self.detector = apriltag("tagStandard41h12")
        # self.req = self.picam2.capture_request(flush=True)
        self.rate = 1/56.03
        self.d = 40

    def setCamera(self):
        mode = self.picam2.sensor_modes[1]
        self.config = self.picam2.create_still_configuration(lores={"size": (648, 648), "format": "RGB888"}, sensor={'output_size': mode['size']}, encode="lores", buffer_count=1, queue=False)

        self.picam2.configure( self.config )
        self.picam2.start()
        self.picam2.set_controls({
            "ScalerCrop": (1104, 96, 2400, 2400),
            "ExposureTime": 2000,
            "Contrast": 1.2,
        })
        time.sleep(.5)

    def getBallCoordinate(self):
        self.getFrame()
        Gpoint = self.detectTagCoordinate(self.frame)
        Bpoint = self.detectBallCoordinate(self.frame)
        if (Gpoint is None) or (Bpoint is None):
            return None
        return self.getBallLocalCoordinate(Gpoint, Bpoint)

    def getFrame(self):
        # self.frame = self.req.make_array("lores")
        # self.frame = cv2.cvtColor(self.frame, cv2.COLOR_RGB2GRAY)
        self.frame = self.picam2.capture_array("lores")
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_RGB2GRAY)
    
    def detectTagCoordinate(self, frame):
        detections = self.detector.detect(frame)
        Gpoint = {}
        for tag in detections:
            if tag['id'] == 7:
                G1 = tag['center']
                Gpoint['G1'] = G1
                continue
            if tag['id'] == 19:
                G2 = tag['center']
                Gpoint['G2'] = G2
                continue
            if tag['id'] == 23:
                G3 = tag['center']
                Gpoint['G3'] = G3
                continue
            if tag['id'] == 37:
                G4 = tag['center']
                Gpoint['G4'] = G4

        if ( len(Gpoint) < 3):
            return None

        if ( len(Gpoint) == 4):
            G0 = ( G1 + G2 + G3 + G4 ) / 4
            Gpoint['G0'] = G0
            return Gpoint

        if ( ('G1' not in Gpoint) or ('G3' not in Gpoint) ):
            G0 = ( G2 + G4 ) / 2
        elif ( ('G2' not in Gpoint) or ('G4' not in Gpoint) ):
            G0 = ( G1 + G3 ) / 2
        Gpoint['G0'] = G0
        return Gpoint
    
    def detectBallCoordinate(self, frame):
        circles = cv2.HoughCircles(
            frame,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=100,    # 10
            param1=200,
            param2=10,  # 20
            minRadius=12,
            maxRadius=15,
        )
        if (circles is not None):
            return circles[0][0][0:2]
        return None

    def getBallLocalCoordinate(self, Gpoint, Bpoint):
        if ( 'G1' not in Gpoint ):
            vec_xaxis = Gpoint['G4'] - Gpoint['G3']
            vec_yaxis = Gpoint['G2'] - Gpoint['G3']
        elif ( 'G2' not in Gpoint ):
            vec_xaxis = Gpoint['G4'] - Gpoint['G3']
            vec_yaxis = Gpoint['G1'] - Gpoint['G4']
        elif ( 'G3' not in Gpoint ):
            vec_xaxis = Gpoint['G1'] - Gpoint['G2']
            vec_yaxis = Gpoint['G1'] - Gpoint['G4']
        elif ( 'G4' not in Gpoint ):
            vec_xaxis = Gpoint['G1'] - Gpoint['G2']
            vec_yaxis = Gpoint['G2'] - Gpoint['G3']
        else:
            vec_xaxis = ( Gpoint['G1'] - Gpoint['G2'] + Gpoint['G4'] - Gpoint['G3'] ) / 2
            vec_yaxis = ( Gpoint['G1'] - Gpoint['G4'] + Gpoint['G2'] - Gpoint['G3'] ) / 2
        
        scale = self.d / np.linalg.norm( vec_xaxis ) + self.d / np.linalg.norm( vec_yaxis )
        vec_xaxis = vec_xaxis / np.linalg.norm(vec_xaxis)
        vec_yaxis = vec_yaxis / np.linalg.norm(vec_yaxis)
        vec_G0_P  = Bpoint - Gpoint['G0']
        ball_x = np.dot( vec_xaxis, vec_G0_P ) * scale
        ball_y = np.dot( vec_yaxis, vec_G0_P ) * scale
        return np.array([ball_x, ball_y])

