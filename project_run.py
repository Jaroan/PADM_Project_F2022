from __future__ import print_function

from posixpath import join
import numpy as np

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


from utils import  get_collision_fn_franka, execute_trajectory, draw_sphere_marker

from pybullet_tools.utils import connect, disconnect, draw_circle,  get_movable_joint_descendants, wait_for_user, wait_if_gui, get_joint_name\
	 joint_from_name, get_joint_positions, set_joint_positions, get_joint_info, get_link_pose, link_from_name, get_joint_limits,\
	  body_from_name, get_euler, get_links, get_link_name,  single_collision, get_custom_limits, interval_generator, get_movable_joints

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply ,euler_from_quat, quat_from_euler, \
	 get_pose, get_point, get_links, create_box, set_all_static, WorldSaver, create_plane, set_camera, set_camera_pose,\
	   stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, \
	   set_point, get_function_name,  dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed\

from pybullet_tools.utils import CIRCULAR_LIMITS, COLOR_FROM_NAME, set_joint_position,  interpolate_poses, create_attachment

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
	ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
	BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
	STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

world = World(use_gui=True)


def main():



if __name__ == "__main__":

	problem_file = "problem.pddl"
	domain_file = "domain.pddl"
