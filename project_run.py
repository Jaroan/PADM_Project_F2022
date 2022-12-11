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
from pddl_parser.PDDL import PDDL_Parser
import sys, time
import argparse

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])


from utils import get_sample_fn,round_if_float,rounded_tuple, get_collision_fn_franka, execute_trajectory, draw_sphere_marker

from planner import Planner

# from rrt import rrt_helpers

from pybullet_tools.utils import connect, disconnect, draw_circle,  get_movable_joint_descendants, wait_for_user, wait_if_gui, get_joint_name, \
	 joint_from_name, get_joint_positions, set_joint_positions, get_joint_info, get_link_pose, link_from_name, get_joint_limits,\
	  body_from_name, get_euler, get_links, get_link_name,  single_collision, get_custom_limits, interval_generator, get_movable_joints

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply ,euler_from_quat, quat_from_euler, \
	 get_pose, get_point, get_links, create_box, set_all_static, WorldSaver, create_plane, set_camera, set_camera_pose,\
	   stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, \
	   set_point, get_function_name,  dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed\

from pybullet_tools.utils import CIRCULAR_LIMITS, COLOR_FROM_NAME, set_joint_position,  interpolate_poses, create_attachment

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
	ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
	BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
	STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

UNIT_POSE2D = (0., 0., 0.)

world = World(use_gui=True)

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
sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))

class PlanExecutor():

	def __init__(self, problem_file, domain_file):
		# Parse the PDDL files for activity planning
		self.problem_file = problem_file
		self.domain_file = domain_file
		self.planner = Planner()
		self.plan = self.planner.solve(domain_file, problem_file)

		self.tool_link = link_from_name(world.robot, 'panda_hand')
		self.drawer_joint = joint_from_name(world.kitchen, 'indigo_drawer_top_joint')
		self.drawer_link = link_from_name(world.kitchen, 'indigo_drawer_handle_top')
		self.drawer_links = [link_from_name(world.kitchen, 'indigo_drawer_handle_top'), link_from_name(world.kitchen, 'indigo_drawer_top')]
 
		# self.location_map, self.object_map, self.object_dict, self.current_pos = self.create_maps()

	def drawer(self, command):

		set_joint_positions(world.robot, world.base_joints, (0.7, 0.6, -np.pi/2))

		target_pose = get_link_pose(world.kitchen, link_from_name(world.kitchen, 'indigo_drawer_handle_top'))

		target_pos, target_quat = target_pose
		tool_target_pos = ((target_pos[0]+0.1), target_pos[1], target_pos[2])
		orig_target_euler = euler_from_quat(target_quat)

		tool_target_euler = (-orig_target_euler[0], orig_target_euler[1], (0.5*np.pi))
		tool_target_quat = quat_from_euler(tool_target_euler)

		tool_link = link_from_name(world.robot, 'panda_hand')

		# print("Attempting to get joint angles for target pose: ", target_pos, tool_target_euler)

		target_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, (tool_target_pos, tool_target_quat), max_time=0.05), None)

		if target_joint_angles == None:
			print("Unable to get target joint angles!")

		else:
			print("Setting arm to target joint angles!")
			set_joint_positions(world.robot, world.arm_joints, target_joint_angles)

			target_x, target_y, target_z = tool_target_pos
			
			if command.lower() == 'opened':
				end_pos = ((target_x + 0.4), target_y, target_z)
			else:
				end_pos = ((target_x - 0.4), target_y, target_z)

			target_end_pose = (end_pos, tool_target_quat)

			wait_for_user('Hit enter to open/close drawer')
			
			target_end_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, target_end_pose, max_time=0.05), None)

			if target_end_joint_angles == None:
				print("Unable to get target end joint angles!")
			
			else:
				set_joint_positions(world.robot, world.arm_joints, target_end_joint_angles)

				if command.lower() == 'opened':
					drawer_pos = 0.4
				else:
					drawer_pos = 0

				set_joint_position(world.kitchen, joint_from_name(world.kitchen, 'indigo_drawer_top_joint'), drawer_pos)

		target_end_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, self.tool_link, target_end_pose, max_time=0.05), None)

		set_joint_positions(world.robot, world.arm_joints, target_end_joint_angles)

		# if command.lower() == 'opened':
		# 	drawer_pos = 0.4
		# elif command.lower() == 'closed':
		# 	drawer_pos = 0

		# set_joint_position(world.kitchen, joint_from_name(world.kitchen, 'indigo_drawer_top_joint'), drawer_pos)



	def sugar_move(self):
		object_dict = {
			'potted_meat_can1': (0.7, 0.3, -np.pi/2),
			'sugar_box0': (0.75, 0.65 , -np.pi/2)
		}
		
		wait_for_user('Going to set base in front of sugar box')
		self.sugar_arm = (-0.1, 1.25, 0, -0.6, 0 ,3.04 , 0.741)
		self.sugar_base = (0.8, 0.5, -3.14)

		set_joint_positions(world.robot, world.base_joints, self.sugar_base)
		set_joint_positions(world.robot, world.arm_joints, self.sugar_arm )

		wait_for_user('Grasping sugar box')
		sugar_box_att = create_attachment(world.robot, link_from_name(world.robot, 'panda_hand'), world.get_body(sugar_box))

		wait_for_user('Attempting to move sugar box')
		point2_arm = (3.14, 1.25, 0, -0.6, 0 ,3.04 , 0.741)
		set_joint_positions(world.robot, world.arm_joints, point2_arm)
		sugar_box_att.assign()

		wait_for_user('Attempting to move sugar box')
		point2_arm = (-1.0, 1.00, 0, -0.6, 0 ,3.04 , 0.741)
		set_joint_positions(world.robot, world.arm_joints, point2_arm)
		sugar_box_att.assign()

	def spam_move(self):

		
		object_dict = {
			'potted_meat_can1': (0.7, 0.3, -np.pi/2),
			'sugar_box0': (0.75, 0.65 , -np.pi/2)
		}

		# user_obj = input("What object would you like to locate? ")
		# user_obj = user_obj.strip()
		user_obj = 'potted_meat_can1'
		try:
			link = link_from_name(world.kitchen, 'potted_meat_can1')
			pose = get_link_pose(world.kitchen, link)
			print("Location ", user_obj, " has pose: ", pose)

		except ValueError as e:
			try:
				body = world.get_body(user_obj)
				coord = get_pose(body)[0]
				euler_angles = get_euler(body)
				print("Object ", user_obj, " has coordinates: ", coord)
				print("Euler angles for this object are: ", euler_angles)
			
			except ValueError as e:
				print("Error getting coordinates for the following link: ", e, " Exiting!")
				sys.exit(1)

		set_joint_positions(world.robot, world.base_joints, object_dict[user_obj])

		tool_link = link_from_name(world.robot, 'panda_hand')

		obj_lower, obj_upper = get_aabb(body)
		obj_height = obj_upper[2] - obj_lower[2]

		print("Object aabb is: ", obj_lower, obj_upper)
		
		user_selection = ''
		x_shift_obj_orig = -0.01
		y_shift_obj_orig = -0.13
		# tool_backoff_dist = 0.1 #Distance to back the tool off the object
		x_backoff = (x_shift_obj_orig*np.cos(euler_angles[2])) - (y_shift_obj_orig*np.sin(euler_angles[2]))
		y_backoff = (x_shift_obj_orig*np.sin(euler_angles[2])) + (y_shift_obj_orig*np.cos(euler_angles[2]))

		tool_euler_angles = (euler_angles[0], (-0.5*np.pi), -euler_angles[2])
		coord = ((coord[0]+x_backoff), (coord[1]+y_backoff), (coord[2]+(obj_height*3/4))) #Raise z by 0.05 to be at the right height to grip object
		pose = (coord, quat_from_euler(tool_euler_angles))

		print("Calculated tool pose: ", pose)
		wait_for_user("Setting tool pose now, press enter to confirm")
		# set_tool_pose(world, pose)

		wait_for_user('Grasping spam box')
		spam_box_att = create_attachment(world.robot, link_from_name(world.robot, 'panda_hand'), world.get_body('potted_meat_can1'))

		wait_for_user('Attempting to move spam box')


		while user_selection != 'end':
			
			success = False

			for i in range(10):
				conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)

				if conf != None:
					success = True
					break

				if i == 9:
					print("Unable to run inverse kinematics successfully!")

			if success:
				print("The joint angles to achieve this pose is: ", conf)

				set_joint_positions(world.robot, world.arm_joints, conf)

			user_selection = input(" da 'End' or continue? ")
			user_selection.strip()




	def run(self):

		if type(self.plan) is list:
			print('plan:')
			for act in self.plan:
				print(act.name + ' ' + ' '.join(act.parameters))
		else:
			print('No plan was found')
			exit(1)

		self.arm_plan_dict = dict()
		self.base_plan_dict = dict()

		for i, action in enumerate(self.plan):
			print("Step", i+1, action.parameters[0], action.name, "to do ",action.parameters[1:])
			print(str(action.name)+" "+str(action.parameters[1]))
			if str(action.name)+" "+str(action.parameters[1]) == "move counter":
				# print(str(action.name)+str(action.parameters[1]), " 0 0",base_position_list["counter"])
				self.arm_plan_dict[action.name+str(action.parameters[1])] = object_postion_dict["spam"]
				self.base_plan_dict[action.name+str(action.parameters[1])] = base_position_list["counter"]
			elif  str(action.name)+" "+str(action.parameters[1]) == "opendrawer drawer" or str(action.name)+" "+str(action.parameters[1]) == "closedrawer drawer":
				self.arm_plan_dict[action.name+str(action.parameters[1])] = object_postion_dict["spam"]
				self.base_plan_dict[action.name+str(action.parameters[1])] = base_position_list["drawer"]

			elif  str(action.name)+" "+str(action.parameters[1]) == "pickuptop sugar":
				self.arm_plan_dict[action.name+str(action.parameters[1])] = object_postion_dict["sugar"]
				self.base_plan_dict[action.name+str(action.parameters[1])] = base_position_list["counter"]

			elif  str(action.name)+" "+str(action.parameters[1]) == "pickuptop spam":
				self.arm_plan_dict[action.name+str(action.parameters[1])] = object_postion_dict["spam"]
				self.base_plan_dict[action.name+str(action.parameters[1])] = base_position_list["counter"]


			elif  str(action.name)+" "+str(action.parameters[1]) == "droptop sugar":
				self.arm_plan_dict[action.name+str(action.parameters[1])] = object_postion_dict["spam"]
				self.base_plan_dict[action.name+str(action.parameters[1])] = base_position_list["counter"]

			elif  str(action.name)+" "+str(action.parameters[1]) == "dropdrawer spam":
				self.arm_plan_dict[action.name+str(action.parameters[1])] = object_postion_dict["spam"]
				self.base_plan_dict[action.name+str(action.parameters[1])] = base_position_list["drawer"]

			# print(self.arm_plan_dict)
			# print(self.base_plan_dict)
			# wait_for_user()

		for i, action in enumerate(self.plan):
			print("Step", i+1, action.parameters[0], action.name, "to do ",action.parameters[1:])
			print(str(action.name)+" "+str(action.parameters[1]))
			if str(action.name)+" "+str(action.parameters[1]) == "move counter":
				# print(str(action.name)+str(action.parameters[1]), " 0 0",base_position_list["counter"])
				set_joint_positions(world.robot, world.arm_joints, self.arm_plan_dict[action.name+str(action.parameters[1])])
				set_joint_positions(world.robot, world.base_joints, self.base_plan_dict[action.name+str(action.parameters[1])])

			elif  str(action.name)+" "+str(action.parameters[1]) == "opendrawer drawer" or str(action.name)+" "+str(action.parameters[1]) == "closedrawer drawer":
				set_joint_positions(world.robot, world.arm_joints, self.arm_plan_dict[action.name+str(action.parameters[1])])
				set_joint_positions(world.robot, world.base_joints, self.base_plan_dict[action.name+str(action.parameters[1])])
				self.drawer("opened")

			elif  str(action.name)+" "+str(action.parameters[1]) == "pickuptop sugar":
				set_joint_positions(world.robot, world.arm_joints, self.arm_plan_dict[action.name+str(action.parameters[1])])
				set_joint_positions(world.robot, world.base_joints, self.base_plan_dict[action.name+str(action.parameters[1])])
				self.sugar_move()

			elif  str(action.name)+" "+str(action.parameters[1]) == "pickuptop spam":
				self.arm_plan_dict[action.name+str(action.parameters[1])] = object_postion_dict["spam"]
				self.base_plan_dict[action.name+str(action.parameters[1])] = base_position_list["counter"]
				self.spam_move()


			elif  str(action.name)+" "+str(action.parameters[1]) == "droptop sugar":
				self.arm_plan_dict[action.name+str(action.parameters[1])] = object_postion_dict["spam"]
				self.base_plan_dict[action.name+str(action.parameters[1])] = base_position_list["counter"]

			elif  str(action.name)+" "+str(action.parameters[1]) == "dropdrawer spam":
				self.arm_plan_dict[action.name+str(action.parameters[1])] = object_postion_dict["spam"]
				self.base_plan_dict[action.name+str(action.parameters[1])] = base_position_list["drawer"]
				self.drawer("closed")


			wait_for_user()

object_postion_dict = {"sugar": (-0.1, 1.25, 0, -0.6, 0 ,3.04 , 0.741), \
						"spam": (0.267, 0.990, -0.497,-0.270, -0.653, -0.270, 0.653)}


base_position_list = {"drawer" : (0.7, 0.6, -3.14),\
					"counter": (0.8, 0.5, -3.14)}


def main(executor):

	print('Random seed:', get_random_seed())
	print('Numpy seed:', get_numpy_seed())

	np.set_printoptions(precision=3, suppress=True)

	# print("worls", world)

	# wait_for_user()
	world._update_initial()
	tool_link = link_from_name(world.robot, 'panda_hand')
	joints = get_movable_joints(world.robot)

	sample_fn = get_sample_fn(world.robot, world.arm_joints)


	executor.run()
	print("Destroying world object")
	world.destroy()


if __name__ == "__main__":

	problem_file = "problem.pddl"
	domain_file = "domain.pddl"

	executor = PlanExecutor(problem_file, domain_file)

	main(executor)
