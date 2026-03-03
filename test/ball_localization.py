import cv2
import numpy as np
import time
import test_param

detections = test_param.detections
ball_position = test_param.ball_position


# assign coordination to each point




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

d = 20

P = np.array(ball_position[0:2])


G0 = (G1 + G2 + G3 + G4) / 4

Gpoint = [G0, G1, G2, G3, G4]


vec_G2_G1 = G1 - G2
vec_G0_P = P - G0
vec_G4_G1 = G1 - G4


ball_x = np.dot( vec_G2_G1 , vec_G0_P ) / (2*d)
ball_y = np.dot( vec_G4_G1 , vec_G0_P ) / (2*d)

ball = np.array([ball_x, ball_y])
print(ball)

print(vec_G2_G1)
print(vec_G4_G1)

