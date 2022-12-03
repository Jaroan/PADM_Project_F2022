from __future__ import print_function
#from dataclasses import dataclass

import os, sys, argparse
import numpy as np
import pybullet as p

import gitmodules
__import__('padm-project-2022f') 

from planner import Planner

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, interval_generator, get_link_pose, interpolate_poses

from src.world import World


#my actual imports! 
from pybullet_tools.utils import get_joint_positions, get_link_name, get_links, get_link_pose, set_joint_positions, sub_inverse_kinematics, plan_cartesian_motion
import numpy as np

''' NOTES:
sub_inverse_kinematics(robot, first_joint, target_link, target_pose, **kwargs)
plan_cartesian_motion(robot, first_joint, target_link, waypoint_poses,
                          max_iterations=200, custom_limits={}, **kwargs):


'''

#generate position based on joint limits
def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn


#start world
world = World(use_gui=False)
robot = world.robot

#base spawn position
spawn_position = np.array([.85, .8, np.pi])
set_joint_positions(world.robot, world.base_joints, spawn_position)

#print(get_joint_positions(world.robot, world.base_joints))
#print(get_joint_positions(world.robot, world.arm_joints))

#inverse kinematics help 
tool_link = link_from_name(world.robot, 'panda_hand')

#first_joint = link_from_name(world.robot, 'x')
print('tool link', tool_link)
#print('test Joints', [link_from_name(world.robot, joint) for joint in world.robot])
#first_joint = 13

first_joint = 12
tool_link = 18

target_pose = list((-0.008500000461935997, 1.2109999656677246, -0.6542999744415283)) #drawer location

print('sub IK:', sub_ik_test_case := sub_inverse_kinematics(robot, first_joint, tool_link, target_pose) )

#print('ik:', test := p.calculateInverseKinematics(world.robot, tool_link, target_pose))

for joint in world.gripper_joints:
    print(joint)



#set_joint_positions(world.robot, world.arm_joints, test)

wait_for_user()


def get_config_from_position():
    
    return None

def motion_positions():
    #initialize pose dictionary
    pose = {'drawer_closed':[-0.008500000461935997, 1.2109999656677246, -0.6542999744415283],
            'drawer_open':[],
            'counter':[],
            'stove':[]
            }
    
    #create and setup plans! 
    planner = Planner()
    actions = planner.solve('domain.pddl', 'problem.pddl')
    plan = []
    for act in actions:
        action = [act.name, list(act.parameters)]
        plan.append(action)
    
    ""
    positions = []
    visited = []


    for i in range(len(plan)):
        visited.append(plan[i][0])
        #print(plan[i])
        if plan[i][0] == 'move':
            print(plan[i])

            #if open drawer was last on the visited list, then go to open drawer configuration
                #motion_position('open_drawer')
            #if close drawer is on the visited list, then go to closer drawer configuration
            #motion_position('open_drawer')
            #else go to close drawer configuration
        else:
            positions.append(plan[i][0])
            #figure out how to pickup/drop things lol

        
    return positions

#motion_positions()

'''
obstacles = get_links(world.kitchen)
print(len(obstacles))
obstacle_names = [get_link_name(world.kitchen, link) for link in obstacles]

#'indigo_drawer_handle_bottom'] = 59,
#'indigo_drawer_handle_top' = 57
# indigo_drawer_top = 56

#print(obstacle_names)

#wait_for_user()

print(get_link_name(world.kitchen, 56))
test = link_from_name(world.kitchen, 'indigo_drawer_top')
print('indigo_drawer_top', test)
print(get_link_pose(world.kitchen, test))
'''