from scenic.simulators.isaacsim.behaviors import *
from scenic.simulators.isaacsim.utils.utils import getPreexistingObj

param environmentUSDPath = "C:/isaacsim_assets/Assets/Isaac/4.5/Isaac/Environments/Simple_Room/simple_room.usd"
param environmentMeshPath = localPath("../../../assets/meshes_converted/simple_room_usd.gltf")
param environmentInfoPath = localPath("usd_room_bb.json")
param numToys = 10

model scenic.simulators.isaacsim.model

class Toy(IsaacSimObject):
    shape: Uniform(BoxShape(), CylinderShape(), ConeShape(), SpheroidShape())
    width: 0.1
    length: 0.1
    height: 0.1
    density: 100
    color: [1, 0.502, 0]

floor = getPreexistingObj('Towel_Room01_floor_bottom')

ego = new Create3 on floor, with color (1, 0, 0), with behavior KeepMoving

for _ in range(globalParameters.numToys):
    new Toy on floor