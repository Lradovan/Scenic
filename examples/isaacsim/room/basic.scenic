model scenic.simulators.isaacsim.model
from scenic.simulators.isaacsim.utils.utils import isPlanar
from scenic.simulators.isaacsim.behaviors import *
from scenic.core.utils import repairMesh
import trimesh
from trimesh.transformations import decompose_matrix
import json
import math
import tempfile

# load an environment asset that has been converted to gltf
scene = trimesh.load(localPath("../../../assets/meshes_converted/simple_room_usd.gltf"))

# load a json file with locations of all meshes in the environment
with open(localPath('usd_room_bb.json')) as json_file:
    transforms = json.load(json_file)

class Box(DynamicObject):
    width: 20
    height: 20
    length: 20
    density: 50
    color: Uniform([1, 0.502, 0], [1, 0, 0], [0, 1, 1], [1, 0, 1])

nodes = {}

for node_name in scene.graph.nodes_geometry:
    if node_name in transforms:
        mesh = scene.geometry[node_name]

        if isPlanar(mesh):
            continue
        else:
            print("adding mesh", node_name) 
        #matrix = scene.graph.get('world', node_name)[0]
        #scale, shear, angles, trans, persp = decompose_matrix(matrix)

        trans = np.array(transforms[node_name]) * 100

        obj = new StaticObject with position trans,
                with allowCollisions True, 
                with shape MeshShape(repairMesh(mesh)), 
                with color Uniform([1, 0.502, 0], [1, 0, 0], [0, 1, 1], [1, 0, 1])

        nodes[node_name] = obj

for _ in range(5):
    new Box on nodes['table_low']