from scenic.simulators.isaacsim.actions import *
import numpy as np

behavior FollowPath():

    while True:
        take advance()

# for the create3 wheeled robot
behavior KeepMoving():

    threshold = .01
    while True:
        if np.linalg.norm(self.speed) < threshold:
            for i in range(100):
                take setMoveWheeled(-.2, 0)
            for i in range(50):
                take setMoveWheeled(0, np.pi)
        else:
            take setMoveWheeled(.2, 0)

# for the kaya robot
behavior RandomMovement():
    i = 0

    while True:
        if i >= 0 and i < 500:
            take setMoveHolonomic(-0.7, 0.0, 0.0)
        elif i >= 500 and i < 1000:
            take setMoveHolonomic(0.0, 0.4, 0.0)
        elif i >= 1000 and i < 1100:
            take setMoveHolonomic(0.0, 0.0, 0.05)
        elif i == 1200:
            i = 0
        i += 1

# kaya robot
behavior DriveForward():

    while True:
        take setMoveHolonomic(-0.7, 0.0, 0.0)