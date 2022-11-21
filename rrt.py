from __future__ import print_function

from posixpath import join
import numpy as np

# import random
### YOUR IMPORTS HERE ###
import random as rd
import time
from decimal import Decimal
from copy import deepcopy


import os
import gitmodules # need to pip install
__import__('padm-project-2022f') 
import sys
import argparse

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])
# from utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, draw_circle,  get_movable_joint_descendants, wait_for_user, wait_if_gui, joint_from_name, get_joint_positions, set_joint_positions, get_joint_info, get_link_pose, link_from_name




from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
	ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
	BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
	STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
	name = name_from_type(ycb_type, idx)
	world.add_body(name, color=np.ones(4))
	pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
	return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
	x, y, yaw = pose2d
	body = world.get_body(entity_name)
	surface_aabb = compute_surface_aabb(world, surface_name)
	z = stable_z_on_aabb(body, surface_aabb)
	pose = Pose(Point(x, y, z), Euler(yaw=yaw))
	set_pose(body, pose)
	return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
	lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
	generator = interval_generator(lower_limits, upper_limits, **kwargs)
	def fn():
		return tuple(next(generator))
	return fn

def main():
	print('Random seed:', get_random_seed())
	print('Numpy seed:', get_numpy_seed())

	np.set_printoptions(precision=3, suppress=True)
	world = World(use_gui=True)
	sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
	spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
	wait_for_user()
	world._update_initial()
	tool_link = link_from_name(world.robot, 'panda_hand')
	joints = get_movable_joints(world.robot)

	print("Joints", joints)
	print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
	print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
	sample_fn = get_sample_fn(world.robot, world.arm_joints)
	print("Going to use IK to go from a sample start state to a goal state\n")
	tool_link = link_from_name(world.robot, 'panda_hand')
	joints = get_movable_joints(world.robot)
	kitchen = get_movable_joints(world.kitchen)

	print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
	print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints]) 
	print('Gripper Joints', [get_joint_name(world.robot, joint) for joint in world.gripper_joints]) 
	print('Kitchen Joints', [get_joint_name(world.kitchen, kitchen) for kitchen in world.kitchen_joints]) 
	# define active DoFs
	joint_names =('panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7')
	# joint_names = (1,2,3,4,5,6,7)
	joint_idx = [joint_from_name(world.robot, joint) for joint in joint_names]

	# parse active DoF joint limits
	joint_limits = {joint_names[i] : (get_joint_info(world.robot, joint_idx[i]).jointLowerLimit, get_joint_info(world.robot, joint_idx[i]).jointUpperLimit) for i in range(len(joint_idx))}

	collision_fn = get_collision_fn_franka(world.robot, joint_idx, list(obstacles.values()))
	# Example use of collision checking
	# print("Robot colliding? ", collision_fn((0.5, 1.19, -1.548, 1.557, -1.32, -0.1928)))

	start_config = tuple(get_joint_positions(robots['pr2'], joint_idx))
	goal_config = (0.5, 0.33, -1.548, 1.557, -1.32, -0.1928)
	path = []

	### 
	start_time = time.time()


	print("joint limits: ")
	for i in range(len(joint_limits)):
		print(joint_limits[joint_names[i]])
	print(joint_limits)


    joints_start = [joint_limits[joint_names[i]][0] for i in range(len(joint_limits))]
    joints_scale = [abs(joint_limits[joint_names[i]][0] - joint_limits[joint_names[i]][1]) for i in range(len(joint_limits))]
    print("joint start: ",joints_start)
    print("joint scale: ",joints_scale)
    # print("joints scale: ",joints_scale)

    ###### Modify Parameters ######
    """
    Edit the part below
    """

    # KEY: node : a tuple, NOT list! 
    step_size = 0.05 #rad for each joint (revolute)
    goal_bias_prob = 0.1 # goal_bias: 10%
    goal_node = goal_config
    root_parent = (-1,-1,-1,-1,-1,-1)
    # Total: 6 DOF 
    K = 3000  # 10000 nodes iter: 81, rrt# 987
    rrt = [start_config]     # a list of config_nodes
    parents = {} # a dictionary key: tuple(a config), value: tuple(parent's config)
    parents[start_config] = root_parent

    # goal_threshold = 0.5
    goal_threshold = 0.4
    findpath = False

	# obstacles = [plane] # TODO: collisions with the ground

	# dump_body(robot)
	# print('Start?')
	# wait_for_user()

	# info = PANDA_INFO
	# tool_link = link_from_name(robot, 'panda_hand')
	# draw_pose(Pose(), parent=robot, parent_link=tool_link)
	# joints = get_movable_joints(robot)
	# print('Joints', [get_joint_name(robot, joint) for joint in joints])
	# check_ik_solver(info)

	# sample_fn = get_sample_fn(robot, joints)

	# print("Going to operate the base without collision checking")
	# for i in range(100):
	# 	goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
	# 	set_joint_positions(world.robot, world.base_joints, goal_pos)
	# 	if (i % 30 == 0):
	# 		wait_for_user()
	wait_for_user()
	world.destroy()

if __name__ == '__main__':
	main()

