from __future__ import print_function

import os, sys, argparse
#import numpy as np

import gitmodules # need to pip install
__import__('padm-project-2022f') 

#sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

import pybullet

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly


#pybullet.connect(pybullet.GUI)
#pybullet.connect(pybullet.DIRECT)
#robot = pybullet.loadURDF("padm-project-2022f/models/franka_description/robots/panda_arm_hand_on_carter.urdf")


world = World(use_gui=False)

tool_link = link_from_name(world.robot, 'panda_hand')
joints = get_movable_joints(world.robot)
kitchen = get_movable_joints(world.kitchen)

print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints]) 
print('Gripper Joints', [get_joint_name(world.robot, joint) for joint in world.gripper_joints]) 
print('Kitchen Joints', [get_joint_name(world.kitchen, kitchen) for kitchen in world.kitchen_joints]) 



wait_for_user()
pybullet.disconnect()

