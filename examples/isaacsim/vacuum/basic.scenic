model scenic.simulators.isaacsim.model
from scenic.simulators.isaacsim.behaviors import *

param numToys = 3
param duration = 5

class DiningChair(IsaacSimObject):
    shape: MeshShape.fromFile(localPath("../../../assets/meshes/dining_chair.obj.bz2"), initial_rotation=(180 deg, 0, 0))
    density: 670 # Density of solid birch
    width: 0.4
    length: .4
    height: 1
    color: [0.403, 0.278, 0.212]

class DiningTable(IsaacSimObject):
    shape: MeshShape.fromFile(localPath("../../../assets/meshes/dining_table.obj.bz2"))
    density: 670 # Density of solid birch
    width: Range(0.7, 1.5)
    length: Range(0.7, 1.5)
    height: 0.75
    color: [0.403, 0.278, 0.212]

class CoffeeTable(IsaacSimObject):
    shape: MeshShape.fromFile(localPath("../../../assets/meshes/coffee_table.obj.bz2"))
    width: 1.5
    length: 0.5
    height: 0.4
    color: [0.404, 0.278, 0.212]

class Wall(IsaacSimObject):
    width: 5
    physics: False
    gravity: False
    length: 0.04
    height: 0.5
    color: [0.627, 0.627, 0.627]

class Couch(IsaacSimObject):
    shape: MeshShape.fromFile(localPath("../../../assets/meshes/couch.obj.bz2"), initial_rotation=(-90 deg, 0, 0))
    width: 2
    length: 0.75
    height: 0.75
    color: [0.2, 0.2, 1]

class Toy(IsaacSimObject):
    shape: Uniform(BoxShape(), CylinderShape(), ConeShape(), SpheroidShape())
    width: 0.1
    length: 0.1
    height: 0.1
    density: 100
    color: [1, 0.502, 0]

room_region = RectangularRegion(0 @ 0, 0, 5.09, 5.09)
workspace = Workspace(room_region)

floor = new GroundPlane with color (1, 1, 1), with width 5, with length 5

ego = new Create3 on floor, with color (0, 1, 0), with behavior AvoidObstacles

wall_offset = floor.width/2 + 0.04/2 + 1e-4
right_wall = new Wall at (wall_offset, 0, 0.25), facing toward floor
left_wall = new Wall at (-wall_offset, 0, 0.25), facing toward floor
front_wall = new Wall at (0, wall_offset, 0.25), facing toward floor
back_wall = new Wall at (0, -wall_offset, 0.25), facing toward floor

# Create a "safe zone" around the vacuum so that it does not start stuck
safe_zone = CircularRegion(ego.position, radius=1)

# Create a dining room region where we will place dining room furniture
dining_room_region = RectangularRegion(1.25 @ 0, 0, 2.5, 5).difference(safe_zone)

# Place a table with 3 chairs around it
dining_table = new DiningTable contained in dining_room_region, on floor,
    facing Range(0, 360 deg)

chair_1 = new DiningChair behind dining_table by 0.1, on floor,
                facing toward dining_table, with regionContainedIn dining_room_region
chair_2 = new DiningChair ahead of dining_table by 0.1, on floor,
                facing toward dining_table, with regionContainedIn dining_room_region
chair_3 = new DiningChair left of dining_table by 0.1, on floor,
                facing toward dining_table, with regionContainedIn dining_room_region

# Create a living room region where we will place living room furniture
living_room_region = RectangularRegion(-1.25 @ 0, 0, 2.5, 5).difference(safe_zone)

couch = new Couch ahead of left_wall by 0.335,
            on floor, facing away from left_wall

coffee_table = new CoffeeTable ahead of couch by 0.336,
            on floor, facing away from couch

# Add some noise to the positions of the couch and coffee table
mutate couch, coffee_table

# Spawn some toys
for _ in range(globalParameters.numToys):
    new Toy on floor