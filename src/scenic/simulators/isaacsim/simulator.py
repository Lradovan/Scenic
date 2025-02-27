from scenic.core.simulators import Simulation, Simulator
from isaacsim import SimulationApp
from scenic.core.vectors import Vector
from scenic.core.simulators import SimulationCreationError
import scenic.simulators.isaacsim.utils.utils as utils
from scenic.core.regions import MeshVolumeRegion
import math
import asyncio
import tempfile
from os import path
import trimesh

class IsaacSimSimulator(Simulator):

    def __init__(self):
        super().__init__()
        # launch isaac sim - needs to run before any additional imports
        self.client = SimulationApp({"headless": False})

    def createSimulation(self, scene, **kwargs):
        return IsaacSimSimulation(
            scene,
            self.client,
            **kwargs
        )

class IsaacSimSimulation(Simulation):

    def __init__(self, scene, client, *, timestep, **kwargs):

        from omni.isaac.core import World
        from omni.isaac.core.utils.extensions import enable_extension
        enable_extension("omni.kit.asset_converter")

        self.client = client
        timestep = 1.0/60.0 if timestep is None else timestep
        self.world = World(stage_units_in_meters=1.0, physics_dt=timestep, rendering_dt=timestep)
        self.tmpMeshDir = tempfile.mkdtemp()
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        import omni.kit.actions.core
        super().setup()
        action_registry = omni.kit.actions.core.get_action_registry()
        action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        action.execute()
        self.world.play()

    def step(self):
        self.world.step()

    def createObjectInSimulator(self, obj):
        
        isaac_sim_obj = None
        #ground plane or robot
        if obj.get_type() != 'IsaacSimObject':
            isaac_sim_obj = obj.create()
        else: 
            objectScaledMesh = MeshVolumeRegion(
                mesh=obj.shape.mesh,
                dimensions=(obj.width, obj.length, obj.height),
            ).mesh
            obj_file_path = path.join(self.tmpMeshDir, f"{obj.name}.obj")
            usd_file_path = path.join(self.tmpMeshDir, f"{obj.name}.usd")
            trimesh.exchange.export.export_mesh(objectScaledMesh, obj_file_path)
            asyncio.get_event_loop().run_until_complete(utils.convert(obj_file_path, usd_file_path, True))

            isaac_sim_obj = obj.create(usd_path=usd_file_path)

        if not obj.gravity:
            isaac_sim_obj.prim.GetAttribute("physxRigidBody:disableGravity").Set(True)

        try:
            self.world.scene.add(isaac_sim_obj)
        except:
            raise SimulationCreationError(f"Unable to add {obj.name} to world")
        
        # if it is a robot we need to reset the world
        if obj.get_type() == 'Robot':
            self.world.reset()

    def getProperties(self, obj, properties):
        from omni.isaac.core.utils.rotations import quat_to_euler_angles

        if obj.get_type() == "GroundPlane":  # static object 
            return {prop: getattr(obj, prop) for prop in properties}

        isaacObj = self.world.scene.get_object(obj.name)
        position, orientation = isaacObj.get_world_pose()
        x, y, z = position
        yaw, pitch, roll = quat_to_euler_angles(orientation)
        lx, ly, lz = isaacObj.get_linear_velocity()
        ax, ay, az = isaacObj.get_angular_velocity()
        speed = math.hypot(lx, ly, lz)
        angularSpeed = math.hypot(ax, ay, az)

        values = dict(
            position=Vector(x, y, z),
            velocity=Vector(lx, ly, lz),
            speed=speed,
            angularSpeed=angularSpeed,
            angularVelocity=Vector(ax, ay, az),
            yaw=yaw,
            pitch=pitch,
            roll=roll,
        )
        return values
    
    def destroy(self):
       self.client.close()