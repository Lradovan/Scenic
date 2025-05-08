import uuid
import numpy as np
import os
from scenic.simulators.isaacsim.behaviors import *
from scenic.simulators.isaacsim.utils.utils import scenicToIsaacSimOrientation

try:
    from scenic.simulators.isaacsim.simulator import IsaacSimSimulator    # for use in scenarios
    from scenic.simulators.isaacsim.actions import *
    from scenic.simulators.isaacsim.actions import _Robot, _WheeledRobot, _HolonomicRobot
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

    class _Robot: pass
    class _WheeledRobot: pass
    class _HolonomicRobot: pass

class IsaacSimObject:
    name: f"Object_{uuid.uuid4().hex[:8]}"
    usd_path: None

    def is_isaac_sim_object(self): pass

    def _apply_color(self, prim):
        from isaacsim.core.api.materials import PreviewSurface

        if self.color:
            material = PreviewSurface(prim_path=f"/World/material/{self.name}", color=np.array(self.color))
            prim.apply_visual_material(material)

class Environment:
    usd_path: None

    def create(self):
        pass

# used for imovable things, like walls
class StaticObject(IsaacSimObject):

    def get_type(self):
        return "StaticObject"

    def create(self):

        from isaacsim.core.utils import prims
        from isaacsim.core.prims import SingleGeometryPrim

        prim_path = f"/World/{self.name}"

        prim = prims.create_prim(prim_path=prim_path, usd_path=os.path.abspath(self.usd_path))

        obj = SingleGeometryPrim(
            prim_path=prim_path,
            name=self.name,
            position=self.position,
            orientation=scenicToIsaacSimOrientation(self.orientation),
            collision=True,
        )

        self._apply_color(obj)

        return obj

# used for things that need physics applied to them
class DynamicObject(IsaacSimObject):

    mass: None
    density: None

    def get_type(self):
        return "DynamicObject"

    def create(self):

        from isaacsim.core.utils import prims
        from isaacsim.core.prims import SingleRigidPrim
        from omni.physx.scripts import utils

        prim_path = f"/World/{self.name}"

        prim = prims.create_prim(
            prim_path=prim_path,
            usd_path=os.path.abspath(self.usd_path),
        )
        utils.setRigidBody(prim, "convexDecomposition", False)

        # wrap with a rigid prim to be able to simulate it
        obj = SingleRigidPrim(
            prim_path=prim_path, 
            name=self.name,
            position=self.position,
            orientation=scenicToIsaacSimOrientation(self.orientation),
            mass=self.mass,
            density=self.density,
            linear_velocity=self.velocity)

        # apply material to change the color if specified
        self._apply_color(obj)

        return obj 

def create_controller(forward_func, name): 
    from isaacsim.core.api.controllers import BaseController
    
    class Controller(BaseController):
        def __init__(self):
            super().__init__(name=name)
            
        def forward(self, command):
            return forward_func(command)
    
    return Controller

class IsaacSimRobot(IsaacSimObject, _Robot):

    controller: None
    control: None

    def move(self, sim, command):
        robot = sim.world.scene.get_object(self.name)
        robot.apply_action(self.controller.forward(command=command))

    def create(self):

        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.stage import add_reference_to_stage

        if self.control:
            self.controller = create_controller(self.control, f'{self.name}_controller')()

        prim_path = f"/World/{self.name}"

        add_reference_to_stage(os.path.abspath(self.usd_path), prim_path)

        return Robot(
            prim_path=prim_path, 
            name=self.name,
            position=self.position,
            orientation=scenicToIsaacSimOrientation(self.orientation)
        )

    def get_type(self):
        return "Robot"

class Create3(IsaacSimRobot, _WheeledRobot):

    shape: CylinderShape()
    width: 0.335
    length: 0.335
    height: .1

    def move(self, sim, throttle=0, steering=0):
        wheeled_robot = sim.world.scene.get_object(self.name)
        wheeled_robot.apply_wheel_actions(self.controller.forward(command=[throttle, steering]))

    def create(self):
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        from isaacsim.core.api.materials import PreviewSurface
        from isaacsim.storage.native import get_assets_root_path

        self.controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        create3_asset_path = assets_root_path + "/Isaac/Robots/iRobot/create_3.usd"
        prim_path = f"/World/{self.name}"

        prim = WheeledRobot(
            prim_path=prim_path, 
            name=self.name,
            create_robot=True,
            orientation=scenicToIsaacSimOrientation(self.orientation, initial_rotation=[0, 0, 0]),
            position=self.position,
            usd_path=create3_asset_path,
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
        from isaacsim.core.api.materials import PreviewSurface
        from isaacsim.robot.wheeled_robots.controllers.holonomic_controller import HolonomicController
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        from isaacsim.robot.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
        from isaacsim.storage.native import get_assets_root_path

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        kaya_asset_path = assets_root_path + "/Isaac/Robots/Kaya/kaya.usd"
        prim_path = f"/World/{self.name}"

        prim = WheeledRobot(
            prim_path=prim_path,
            name=self.name,
            wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
            create_robot=True,
            usd_path=kaya_asset_path,
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

class GroundPlane(StaticObject):

    width: 5
    length: 5
    height: 0.01

    def create(self):
        from isaacsim.core.api.objects import GroundPlane

        return GroundPlane(
            name=self.name,
            prim_path="/World/GroundPlane", 
            z_position=0, 
            color=np.array(self.color) if self.color else None,
            scale=[self.width, self.length, self.height]
        )