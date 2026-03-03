from BallTracker import *
from BallController import *
from ST3215_Actuator import *
import numpy as np
import threading

ballTracker = BallTracker()
rate = ballTracker.rate

bb = BallBalance()
h = 80

bController = BallController(rate)
act = Actuator("/dev/ttyACM0", ids=[1, 2, 3])
act.set_zero_pos([2028, 2048, 2038])
act.set_direction(np.full(3, -1))
act.set_mode([MotorMode.POS])

act.target_pos_rad([0, 0, 0])

# act.torque_enable()


tiltController = AntiWindupPID(rate, Kp=0.001, Ki=0.000, Kd=0.0005, windup_limits=(-.02, 0.02), output_limits=(-.0524, .0524))


test_time = 5
process_timer = Timer()
run_timer = Timer()
counter = 0
run_count = int(test_time/ballTracker.rate)

reference = np.array([0, 0])

print("Start ball tracking")

while True:
    if ( process_timer.lap() < rate ):
        continue
    # if ( counter > run_count ):
    #     break
    counter = counter + 1
    process_timer.reset()

    ballCoordinate = ballTracker.getBallCoordinate()
    print("Ball coordinate: ", ballCoordinate)
    if (ballCoordinate is not None):
        if ( np.linalg.norm(ballCoordinate) < 1 ):
            continue
        # angle = np.deg2rad(bController.update(np.array([0, 0]), ballCoordinate))
        tilt_angle = tiltController.update(np.linalg.norm(reference), -np.linalg.norm(ballCoordinate))
        # tilt_angle = 0.0005 * np.linalg.norm(ballCoordinate) 
        # print("Command angle: ", angle)
        # motor_angles = bb.IK(angle[1], angle[0], h)
        axis = np.array([ballCoordinate[1], -ballCoordinate[0], 0])
        print("Tilt axis", axis, ", angle: ", tilt_angle)
        motor_angles = bb.IK(axis, tilt_angle, h)
        print("Motor angles: ", motor_angles)
        act.target_pos_rad(motor_angles)



print("Elapsed time: ", run_timer.lap())

