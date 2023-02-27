#!/usr/bin/env python
from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch
from isaacgym.torch_utils import *
import math
import rospkg
import rospy

# Initialize gym
gym = gymapi.acquire_gym()

# configure sim
sim_params = gymapi.SimParams()
sim_params.up_axis = gymapi.UP_AXIS_Z 
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)

# init ROS
rospy.init_node("icra2023_quadruped_map")
ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()
PKG_NAME = 'ICRA2023_Quadruped_Competition'
PKG_PATH = r.get_path(PKG_NAME)
use_gpu = rospy.get_param('~use_gpu', True)

# configure dynamic engine
if use_gpu:
    sim_params.substeps = 1
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 4
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.num_threads = 0
    sim_params.physx.use_gpu = True
    sim_params.use_gpu_pipeline = True
else: 
    sim_params.use_gpu_pipeline = False

sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

if sim is None:
    raise Exception("Failed to create sim")

# Create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise Exception("Failed to create viewer")

# Add ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)
gym.add_ground(sim, plane_params)

# Load map asset
asset_root = PKG_PATH.rsplit('/',2)[0]
asset_file = "/".join(PKG_PATH.rsplit('/',2)[1:]) + "/urdf/map.urdf"

asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
# asset_options.flip_visual_attachments = True
# asset_options.armature = 0.01
# asset_options.disable_gravity = True

print("Loading asset '%s' from '%s'" % (asset_file, asset_root))
asset = gym.load_asset(
    sim, asset_root, asset_file, asset_options)


# Set up the env grid
num_envs = 1
num_per_row = int(math.sqrt(num_envs))
spacing = 1.0
env_lower = gymapi.Vec3(-spacing, -spacing, 0.0)
env_upper = gymapi.Vec3(spacing, spacing, spacing)

# default  pose
pose = gymapi.Transform()
pose.p = gymapi.Vec3(0, 0, 0)
pose.r = gymapi.Quat(0, 0, 0, 1)

print("Creating %d environments" % num_envs)

envs = []
hand_idxs = []
init_pos_list = []
init_orn_list = []

for i in range(num_envs):
    # Create env
    env = gym.create_env(sim, env_lower, env_upper, num_per_row)
    envs.append(env)

    # Add franka
    asset_handle = gym.create_actor(env, asset, pose, "map", i, 1)


# Point camera at middle env
cam_pos = gymapi.Vec3(5.616360, -9.720163, 6.974301)
cam_target = gymapi.Vec3(0, 0, 0)
middle_env = envs[num_envs // 2 + num_per_row // 2]
gym.viewer_camera_look_at(viewer, middle_env, cam_pos, cam_target)

# ==== prepare tensors =====
# from now on, we will use the tensor API to access and control the physics simulation
gym.prepare_sim(sim)

itr = 0
while not gym.query_viewer_has_closed(viewer):
    itr += 1

    # Update jacobian and mass matrix
    gym.refresh_rigid_body_state_tensor(sim)
    gym.refresh_dof_state_tensor(sim)
    gym.refresh_jacobian_tensors(sim)
    gym.refresh_mass_matrix_tensors(sim)

    # Step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # Step rendering
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, False)
    gym.sync_frame_time(sim)
