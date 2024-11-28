from scenic.simulators.isaacsim.actions import *
import numpy as np

# for the create3 wheeled robot
behavior AvoidObstacles():

    threshold = .01
    while True:
        if np.linalg.norm(self.speed) < threshold:
            for i in range(100):
                take setReverse()
            for i in range(200):
                take setRotate()
        else:
            take setForward()