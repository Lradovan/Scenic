import asyncio
import trimesh
import numpy as np

async def convert(in_file, out_file, load_materials=False):
    # This import causes conflicts when global
    import omni.kit.asset_converter

    def progress_callback(progress, total_steps):
        pass

    converter_context = omni.kit.asset_converter.AssetConverterContext()
    # setup converter and flags
    converter_context.ignore_materials = not load_materials
    # converter_context.ignore_animation = False
    # converter_context.ignore_cameras = True
    # converter_context.single_mesh = True
    # converter_context.smooth_normals = True
    # converter_context.preview_surface = False
    # converter_context.support_point_instancer = False
    # converter_context.embed_mdl_in_usd = False
    # converter_context.use_meter_as_world_unit = True
    # converter_context.create_world_as_default_root_prim = False
    instance = omni.kit.asset_converter.get_instance()
    task = instance.create_converter_task(in_file, out_file, progress_callback, converter_context)
    success = True
    while True:
        success = await task.wait_until_finished()
        if not success:
            await asyncio.sleep(0.1)
        else:
            break
    return success

def planeToMesh(mesh):
    
    normal = mesh.face_normals[0]
    polygon = trimesh.path.polygons.projected(mesh, normal=normal)
    extruded = trimesh.creation.extrude_polygon(polygon, height=.01)

    # fix the orientation
    z_axis = np.array([0, 0, 1])
    rotation = trimesh.geometry.align_vectors(z_axis, normal)
    extruded.apply_transform(rotation)
    
    return extruded
    
def isPlanar(mesh, tolerance=1e-3):

    plane_origin, plane_normal = trimesh.points.plane_fit(mesh.vertices)
    distances = np.abs(np.dot(mesh.vertices - plane_origin, plane_normal))

    return np.all(distances <= tolerance)

def scenicToIsaacSimOrientation(orientation, initial_rotation=None):
    from isaacsim.core.utils.rotations import euler_angles_to_quat

    yaw, pitch, roll = orientation.eulerAngles

    if initial_rotation:
        yaw += initial_rotation[0]
        pitch += initial_rotation[1]
        roll += initial_rotation[2]

    return euler_angles_to_quat([pitch, roll, yaw])

# we use a different environment to not break the simulation app's extensions
# def getShapeFromUSD(usd_path, output_path="temp.obj"):
#     import os
#     env_python_path = os.path.join("env_usd", "Scripts", "python.exe")
#     script_path = os.path.join(os.getcwd(), "extract_mesh.py")

#     subprocess.run(
#         [env_python_path, script_path, usd_path, output_path],
#         check=True,
#         capture_output=True,
#         text=True,
#     )

#     return trimesh.load(output_path)

_prexistingObjs = {}

def _addPreexistingObj(obj):
    _prexistingObjs[obj.name] = obj

def getPreexistingObj(objName):
    return _prexistingObjs[objName]