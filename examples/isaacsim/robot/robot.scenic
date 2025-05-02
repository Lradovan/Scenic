model scenic.simulators.isaacsim.model

# takes a command and returns an ArticulationAction
# https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_adding_controller.html
def create_3_control(command):

    from isaacsim.core.utils.types import ArticulationAction

    wheel_radius = .03 
    wheel_base=0.1125
    joint_indices = np.array([0, 1]) 

    joint_velocities = [0.0, 0.0]
    joint_velocities[0] = ((2 * command[0]) - (command[1] * wheel_base)) / (2 * wheel_radius)
    joint_velocities[1] = ((2 * command[0]) + (command[1] * wheel_base)) / (2 * wheel_radius)

    return ArticulationAction(joint_velocities=joint_velocities, joint_indices=joint_indices)

# create 3 robot
class Create_3(IsaacSimRobot):
    usd_path: localPath("../../../assets/meshes/create_3.usd")
    shape: MeshShape.fromFile(localPath("../../../assets/meshes_converted/create_3_usd.obj"))
    control: create_3_control