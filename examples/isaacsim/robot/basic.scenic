model scenic.simulators.isaacsim.model
from scenic.simulators.isaacsim.behaviors import *
import scenic.simulators.isaacsim.utils.utils as utils
from robot import *

floor = new GroundPlane with color (1, 1, 1), with width 8, with length 8

class Box(DynamicObject):
    width: .2
    height: .2
    length: .2
    density: 50
    physics: True
    color: Uniform([1, 0.502, 0], [1, 0, 0], [0, 1, 1], [1, 0, 1])

class Pallet(DynamicObject):
    usd_path: localPath("../../../assets/meshes/pallet.usd")
    shape: MeshShape.fromFile(localPath("../../../assets/meshes_converted/pallet_usd.obj"))
    color: [1, .65, 0]
    gravity: True
    physics: True

p1 = new Pallet on floor
p2 = new Pallet on p1
p3 = new Pallet on p2

new Create_3 on floor, with behavior KeepMovingCustom