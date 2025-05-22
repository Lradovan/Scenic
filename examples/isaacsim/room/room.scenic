from scenic.simulators.isaacsim.behaviors import *
from scenic.simulators.isaacsim.utils.utils import getPreexistingObj

param environmentUSDPath = localPath("../../../assets/usd/simple_room_flattened.usd")
param environmentMeshPath = localPath("../../../assets/meshes_converted/simple_room_flattened_usd.gltf")
param environmentInfoPath = localPath("./simple_room_flattened_info.json")
param numToys = 10

model scenic.simulators.isaacsim.model

class Toy(IsaacSimObject):
    shape: Uniform(BoxShape(), CylinderShape(), ConeShape(), SpheroidShape())
    width: 0.1
    length: 0.1
    height: 0.1
    density: 100
    color: [1, 0.502, 0]

floor = getPreexistingObj("/Root/Towel_Room01_floor_bottom_218/Towel_Room01_floor_bottom")

ego = new Create3 on floor, with color (1, 0, 0)#, with behavior KeepMoving

for _ in range(globalParameters.numToys):
   new Toy on floor