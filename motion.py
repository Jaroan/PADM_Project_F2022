from __future__ import print_function
#from dataclasses import dataclass

import os, sys, argparse
import numpy as np
import pybullet

import gitmodules
__import__('padm-project-2022f') 

from planner import Planner

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

#create plans
planner = Planner()
actions = planner.solve('domain.pddl', 'problem.pddl')
plan = []

for act in actions:
    plan.append(act.name + ' ' + ' '.join(act.parameters))

########################################################### useful functions

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn
def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

UNIT_POSE2D = (0., 0., 0.)
def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

########################################################## test imports

from pybullet_tools.utils import get_joint_position, has_joint, get_joint_velocity, get_full_configuration, get_joint_limits, violates_limit, get_collision_data


######################################################### main
#initialize world
world = World(use_gui=False)
robot = world.robot
add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
world._update_initial()

tool_link = link_from_name(world.robot, 'panda_hand')           #gripper links

kitchen = get_movable_joints(world.kitchen)                     #kitchen joints
joints = get_movable_joints(world.robot)                        #gripper links

#print('Kitchen Joints', [get_joint_name(world.kitchen, kitchen) for kitchen in world.kitchen_joints])
#print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints]) 
#print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints]) 
#print('Gripper Joints', [get_joint_name(world.robot, joint) for joint in world.gripper_joints]) 

#pybullet.getOverlappingObjects()


arm_collision = [get_collision_data(robot, base) for base in world.arm_joints] 

test = list(world.static_obstacles)

print(test)

print(world.all_bodies)

goal_pos = translate_linearly(world, 1)
set_joint_positions(world.robot, world.base_joints, goal_pos)



wait_for_user()
pybullet.disconnect()