model scenic.simulators.isaacsim.model
from scenic.simulators.isaacsim.behaviors import *
import scenic.simulators.isaacsim.utils.utils as utils
from robot import *

floor = new GroundPlane with color (1, 1, 1), with width 8, with length 8

class Box(IsaacSimObject):
    width: .2
    height: .2
    length: .2
    density: 50
    color: Uniform([1, 0.502, 0], [1, 0, 0], [0, 1, 1], [1, 0, 1])

new Jetbot on floor, with behavior JetbotDrive