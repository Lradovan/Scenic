from scenic.simulators.isaacsim.behaviors import *
from scenic.simulators.isaacsim.utils.utils import getPreexistingObj

param environmentUSDPath = localPath("../../../assets/usd/warehouse_flattened.usd")
param environmentMeshPath = localPath("../../../assets/meshes_converted/warehouse_flattened_usd.gltf")
param environmentInfoPath = localPath("warehouse_flattened_info.json")

model scenic.simulators.isaacsim.model

floor_piece = getPreexistingObj("/Root/SM_floor58/SM_floor02")

new Create3 on floor_piece, with color (1, 0, 0), with behavior KeepMoving