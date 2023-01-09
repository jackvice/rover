from math import radians
import math
import random
from isaacgym import gymapi
import numpy as np
gym = gymapi.acquire_gym()



sim_params = gymapi.SimParams()
compute_device_id=0
graphics_device_id=0
# get default set of parameters
sim_params = gymapi.SimParams()
# set common parameters
sim_params.dt = 1 / 60
sim_params.substeps = 2
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
# set PhysX-specific parameters
sim_params.physx.use_gpu = True
sim_params.physx.solver_type = 0
sim_params.physx.num_position_iterations = 6
sim_params.physx.num_velocity_iterations = 1
sim_params.physx.contact_offset = 0.00001
sim_params.physx.rest_offset = 0.0
# set Flex-specific parameters
sim_params.flex.solver_type = 5
sim_params.flex.num_outer_iterations = 4
sim_params.flex.num_inner_iterations = 20
sim_params.flex.relaxation = 0.8
sim_params.flex.warm_start = 0.5
# create sim with these parameters
physics_engine=gymapi.SIM_PHYSX
sim = gym.create_sim(compute_device_id, graphics_device_id, physics_engine, sim_params)




# configure the ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1) # z-up!
plane_params.distance = 0
plane_params.static_friction = 0
plane_params.dynamic_friction = 0
plane_params.restitution = 0
# create the ground plane
gym.add_ground(sim, plane_params)



asset_root = "/home/samet/Desktop/urdf_finish/"
asset_file = "roboke.urdf"


asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = False
asset_options.armature = 0.01
#asset_options.override_com = True
#asset_options.override_inertia = True
asset_options.mesh_normal_mode = gymapi.FROM_ASSET
#asset_options.slices_per_cylinder=500
asset_options.vhacd_enabled = True
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = False
asset_options.armature = 0.01
asset_options.vhacd_params.resolution = 300000
asset_options.vhacd_params.max_convex_hulls = 10
asset_options.vhacd_params.max_num_vertices_per_ch = 64
asset_options.vhacd_enabled = True
asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

#insert box cylinder etc in env
width=1
length=1
height=1
radius=1
depth=1
box_asset = gym.create_box(sim, width, height, depth, asset_options)
sphere_asset = gym.create_sphere(sim, radius, asset_options)
capsule_asset = gym.create_capsule(sim, radius, length, asset_options)

# set up the env grid
num_envs = 1
envs_per_row = 5
env_spacing = 2.0
env_lower = gymapi.Vec3(-env_spacing, 0.0, -env_spacing)
env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)

# cache some common handles for later use
envs = []
actor_handles = []

# create and populate the environments
box_asset = gym.create_box(sim, 5,5,0.01, asset_options)

table_pose = gymapi.Transform()
table_pose.p = gymapi.Vec3(0.0,0.0,0.0)
for i in range(num_envs):
    env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
    envs.append(env)
    table_handle = gym.create_actor(env, box_asset, table_pose, "table", i, 0)

    height = random.uniform(1.0, 2.5)

    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0,0,0.3)

    actor_handle = gym.create_actor(env, asset, pose, "MyActor", i, 0)
    actor_handles.append(actor_handle)
    props = gym.get_actor_dof_properties(env, actor_handle)
    props["driveMode"].fill(gymapi.DOF_MODE_EFFORT)
    props["stiffness"].fill(0.0)
    props["damping"].fill(0.0)
    gym.set_actor_dof_properties(env, actor_handle, props)
    vel_targets = np.random.uniform(-math.pi, math.pi, 1).astype('f')
    gym.set_actor_dof_velocity_targets(env, actor_handle, vel_targets)  
    #efforts = np.full(51, 0).astype(np.float32)
    efforts = np.zeros((51,), dtype=np.float32)
    RR=4.0
    RL=-4.0
    FR=-4.0
    FL=4.0
    efforts[0]=-RR
    efforts[13]=FL
    efforts[26]=-FR
    efforts[39]=RL

    gym.apply_actor_dof_efforts(env, actor_handle, efforts)

cam_props = gymapi.CameraProperties()
viewer = gym.create_viewer(sim, cam_props)
while not gym.query_viewer_has_closed(viewer):
    
    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)
