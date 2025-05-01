model scenic.simulators.isaacsim.model
from scenic.simulators.isaacsim.behaviors import *
import scenic.simulators.isaacsim.utils.utils as utils

floor = new GroundPlane with color (1, 1, 1), with width 8, with length 8

create3 = new IsaacSimRobot with usd_path localPath("../../../assets/meshes/create_3.usd"),
                  with shape MeshShape.fromFile(localPath("../../../assets/meshes_converted/create_3_usd.obj")),
                  on floor #, with behavior Drive

# workspace = Workspace(RectangularRegion(0 @ 0, 0, 5.09, 5.09))

class Box(IsaacSimObject):
    width: .2
    height: .2
    length: .2
    density: 50
    physics: False
    color: Uniform([1, 0.502, 0], [1, 0, 0], [0, 1, 1], [1, 0, 1])

class Pallet(IsaacSimObject):
    usd_path: localPath("../../../assets/meshes/pallet.usd")
    shape: MeshShape.fromFile(localPath("../../../assets/meshes_converted/pallet_usd.obj"))
    color: [1, .65, 0]
    gravity: True
    physics: True

new Box on floor

p = new Pallet on floor
p2 = new Pallet on p
p3 = new Pallet on p2

# quad = new IsaacSimRobot with usd_path localPath("../../../assets/meshes/a1.usd"),
#                   with shape MeshShape.fromFile(localPath("../../../assets/meshes_converted/a1_usd.obj")),
#                   on floor 