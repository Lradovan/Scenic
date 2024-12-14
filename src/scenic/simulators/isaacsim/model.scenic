
import uuid
import numpy as np
from scenic.simulators.isaacsim.behaviors import *
from scenic.simulators.isaacsim.utils.utils import scenicToIsaacSimOrientation

try:
    from scenic.simulators.isaacsim.simulator import IsaacSimSimulator    # for use in scenarios
    from scenic.simulators.isaacsim.actions import *
    from scenic.simulators.isaacsim.actions import _WheeledRobot, _QuadrupedRobot, _HolonomicRobot
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
    class _QuadrupedRobot: pass
    class _HolonomicRobot: pass

class IsaacSimObject:
    name: f"Object_{uuid.uuid4().hex[:8]}"
    gravity: True
    physics: True
    mass: None
    density: None
    usd_path: None

    def get_type(self):
        return "IsaacSimObject"

    def create(self, usd_path):

        from omni.isaac.core.utils import prims
        from omni.isaac.core.prims import RigidPrim
        from omni.isaac.core.materials import PreviewSurface
        from omni.physx.scripts import utils

        prim_path = f"/World/{self.name}"

        prim = prims.create_prim(
            prim_path=prim_path,
            usd_path=usd_path,
        )

        utils.setRigidBody(prim, "convexDecomposition", False)

        # wrap with a rigid prim to be able to simulate it
        rigid_prim = RigidPrim(
            prim_path=prim_path, 
            name=self.name,
            position=self.position,
            orientation=scenicToIsaacSimOrientation(self.orientation),
            mass=self.mass,
            density=self.density,
            linear_velocity=self.velocity)

        # apply material to change the color if specified
        if self.color:
            material = PreviewSurface(
                prim_path=f"/World/material/{self.name}",  
                color=np.array(self.color)
            )
            rigid_prim.apply_visual_material(material)

        # do we want physics on this object?
        if self.physics:
            rigid_prim.enable_rigid_body_physics()
        else:
            rigid_prim.disable_rigid_body_physics()

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

    def move(self, sim, throttle=0, steering=0):
        wheeled_robot = sim.world.scene.get_object(self.name)
        wheeled_robot.apply_wheel_actions(self.controller.forward(command=[throttle, steering]))

    def create(self):
        from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
        from omni.isaac.wheeled_robots.robots import WheeledRobot
        from omni.isaac.core.materials import PreviewSurface

        self.controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
        
        usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.2/Isaac/Robots/iRobot/create_3.usd"
        prim_path = f"/World/{self.name}"

        prim = WheeledRobot(
            prim_path=prim_path, 
            name=self.name,
            create_robot=True,
            orientation=scenicToIsaacSimOrientation(self.orientation, initial_rotation=[np.pi/2, 0, 0]),
            position=self.position,
            usd_path=usd_path,
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"])
        
        if self.color:
            material = PreviewSurface(
            prim_path=f"/World/material/{self.name}",  
            color=np.array(self.color) if self.color else None)

            prim.apply_visual_material(material)

        return prim

class Kaya(IsaacSimRobot, _HolonomicRobot):

    width: 0.2
    length: 0.2
    height: 0.2

    def move(self, sim, forward_speed=0, lateral_speed=0, yaw_speed=0):
        wheeled_robot = sim.world.scene.get_object(self.name)
        wheeled_robot.apply_wheel_actions(self.controller.forward(command=[forward_speed, lateral_speed, yaw_speed]))

    def create(self):
        from omni.isaac.core.materials import PreviewSurface
        from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
        from omni.isaac.wheeled_robots.robots import WheeledRobot
        from omni.isaac.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup

        usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/4.2/Isaac/Robots/Kaya/kaya.usd"
        prim_path = f"/World/{self.name}"

        prim = WheeledRobot(
            prim_path=prim_path,
            name=self.name,
            wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
            create_robot=True,
            usd_path=usd_path,
            position=self.position,
            orientation=scenicToIsaacSimOrientation(self.orientation, initial_rotation=[np.pi/2, 0, 0]),
        )

        kaya_setup = HolonomicRobotUsdSetup(
            robot_prim_path=prim_path, 
            com_prim_path=f"/World/{self.name}/base_link/control_offset")

        (
            wheel_radius,
            wheel_positions,
            wheel_orientations,
            mecanum_angles,
            wheel_axis,
            up_axis,
        ) = kaya_setup.get_holonomic_controller_params()

        self.controller = HolonomicController(
            name="holonomic_controller",
            wheel_radius=wheel_radius,
            wheel_positions=wheel_positions,
            wheel_orientations=wheel_orientations,
            mecanum_angles=mecanum_angles,
            wheel_axis=wheel_axis,
            up_axis=up_axis,
        )

        if self.color:
            material = PreviewSurface(
            prim_path=f"/World/material/{self.name}",  
            color=np.array(self.color) if self.color else None)

            prim.apply_visual_material(material)

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