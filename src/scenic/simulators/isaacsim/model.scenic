
import uuid
import numpy as np
from scenic.simulators.isaacsim.behaviors import *
from scenic.simulators.isaacsim.utils.utils import scenicToIsaacSimOrientation

try:
    from scenic.simulators.isaacsim.simulator import IsaacSimSimulator    # for use in scenarios
    from scenic.simulators.isaacsim.actions import *
    from scenic.simulators.isaacsim.actions import _WheeledRobot
    from scenic.simulators.isaacsim.utils.utils import scenicToIsaacSimOrientation
except ModuleNotFoundError:
    # for convenience when testing without the isaacsim package
    from scenic.core.simulators import SimulatorInterfaceWarning
    import warnings
    warnings.warn('the "isaacsim" package is not installed; '
                  'will not be able to run dynamic simulations',
                  SimulatorInterfaceWarning)

    def IsaacSimSimulator(*args, **kwargs):
        """Dummy simulator to allow compilation without the 'isaacsim' package.

        :meta private:
        """
        raise RuntimeError('the "isaacsim" package is required to run simulations '
                           'from this scenario')

    class _WheeledRobot: pass

class IsaacSimObject:
    name: f"Object_{uuid.uuid4().hex[:8]}"
    gravity: True
    physics: True
    mass: None
    density: None

    def get_type(self):
        return "IsaacSimObject"

    def create(self, usd_path):

        from omni.isaac.core.utils import prims
        from omni.isaac.core.prims import RigidPrim
        from omni.isaac.core.materials import PreviewSurface
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.physx.scripts import utils

        prim_path = f"/World/{self.name}"

        prim = prims.create_prim(
            prim_path=prim_path,
            usd_path=usd_path,
        )

        utils.setRigidBody(prim, "convexDecomposition", False)

        material = PreviewSurface(
            prim_path=f"/World/material/{self.name}",  
            color=np.array(self.color) if self.color else None
        )

        # wrap with a rigid prim to be able to simulate it
        rigid_prim = RigidPrim(
            prim_path=prim_path, 
            name=self.name,
            position=self.position,
            orientation=scenicToIsaacSimOrientation(self.orientation),
            scale=[self.width, self.length, self.height],
            mass=self.mass,
            density=self.density,
            linear_velocity=self.velocity)

        # do we want physics on this object?
        if self.physics:
            rigid_prim.enable_rigid_body_physics()
        else:
            rigid_prim.disable_rigid_body_physics()

        # apply the visual material
        rigid_prim.apply_visual_material(material)

        return rigid_prim

class IsaacSimRobot(IsaacSimObject):

    controller: None

    def get_type(self):
        return "Robot"

class Create3(IsaacSimRobot, _WheeledRobot):

    shape: CylinderShape()
    width: 0.335
    length: 0.335
    height: 0.07

    def forward(self, sim):
        wheeled_robot = sim.world.scene.get_object(self.name)
        wheeled_robot.apply_wheel_actions(self.controller.forward(command=[0.1, 0]))

    def backward(self, sim):
        wheeled_robot = sim.world.scene.get_object(self.name)
        wheeled_robot.apply_wheel_actions(self.controller.forward(command=[-0.1, 0]))

    def rotate(self, sim):
        wheeled_robot = sim.world.scene.get_object(self.name)
        wheeled_robot.apply_wheel_actions(self.controller.forward(command=[0.0, np.pi / 4]))

    def create(self):
        import omni.isaac.core.utils.stage as stage_utils
        from omni.isaac.core.robots import Robot
        from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
        from omni.isaac.wheeled_robots.robots import WheeledRobot

        self.controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
        
        usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.2/Isaac/Robots/iRobot/create_3.usd"
        prim_path = f"/World/{self.name}"

        prim = WheeledRobot(
            prim_path=prim_path, 
            name=self.name,
            create_robot=True,
            orientation=scenicToIsaacSimOrientation(self.orientation),
            usd_path=usd_path,
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"])

        return prim

class GroundPlane(IsaacSimObject):

    width: 5
    length: 5
    height: 0.01

    def get_type(self):
        return "GroundPlane"

    def create(self):
        from omni.isaac.core.objects import GroundPlane

        return GroundPlane(
            name=self.name,
            prim_path="/World/GroundPlane", 
            z_position=0, 
            color=np.array(self.color) if self.color else None,
            scale=[self.width, self.length, self.height])

'''
class Cube(IsaacSimObject):

    shape: BoxShape()

    def create(self, orientation):
        from omni.isaac.core.objects import DynamicCuboid

        return DynamicCuboid(prim_path=f"/World/{self.name}",
            name=self.name,
            orientation=orientation,
            scale=(self.width, self.height, self.length)
            position=self.position,
            linear_velocity=self.velocity
        )

class Sphere(IsaacSimObject):

    shape: SpheroidShape()

    def create(self, orientation):
        from omni.isaac.core.objects import DynamicSphere

        return DynamicSphere(prim_path=f"/World/{self.name}",
            name=self.name,
            orientation=orientation,
            scale=(self.width, self.height, self.length)
            position=self.position,
            linear_velocity=self.velocity
        )
'''