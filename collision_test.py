import numpy as np

#from utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_sphere_marker
#from pybullet_tools.utils import connect, disconnect, draw_circle, find, get_movable_joint_descendants, wait_for_user, wait_if_gui, joint_from_name, get_joint_positions, set_joint_positions, get_joint_info, get_link_pose, link_from_name

import random as rd
import time
from decimal import Decimal
from copy import deepcopy

############################################################# my imports

import gitmodules
__import__('padm-project-2022f') 

from src.world import World

from pybullet_tools.utils import joint_from_name, get_joint_info, get_links, get_link_name, get_joint_positions, wait_for_user
#from helper_test import get_collision_fn_franka
from helper_test import get_collision_fn_franka

#############################################################

world = World(use_gui=False)

#specifying joint limits!
joint_names =('panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7')
joint_idx = [joint_from_name(world.robot, joint) for joint in joint_names]
joint_limits = {joint_names[i] : (get_joint_info(world.robot, joint_idx[i]).jointLowerLimit, get_joint_info(world.robot, joint_idx[i]).jointUpperLimit) for i in range(len(joint_idx))}
#print("joint_limits:", joint_limits)

#specifying obstacles!
obstacles = get_links(world.kitchen)
obstacle_names = [get_link_name(world.kitchen, link) for link in obstacles]

collision_fn = get_collision_fn_franka(world.robot, joint_idx, obstacles)

print(get_joint_positions(world.robot, joint_idx))

wait_for_user()
print("Robot colliding? ", collision_fn(get_joint_positions(world.robot, joint_idx)) )
wait_for_user()
