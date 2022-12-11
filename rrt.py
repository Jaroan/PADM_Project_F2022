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
from pybullet_tools.utils import connect, disconnect, draw_circle,  get_movable_joint_descendants, wait_for_user, wait_if_gui, joint_from_name, get_joint_positions, set_joint_positions, get_joint_info, get_link_pose, link_from_name, body_from_name

from utils import  get_collision_fn_franka, execute_trajectory, draw_sphere_marker

from pybullet_tools.utils import joint_from_name, get_euler,get_joint_info, get_links, get_link_name, get_joint_positions, set_joint_positions, single_collision, \
	get_custom_limits, CIRCULAR_LIMITS, interval_generator,get_joint_limits
from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, get_links,get_link_name, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions,set_joint_position, interval_generator, get_link_pose, interpolate_poses,euler_from_quat, quat_from_euler, create_attachment

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
	ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
	BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
	STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

world = World(use_gui=True)
# from collision import get_sample_fn,interpolate_configs,no_collision
UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
	name = name_from_type(ycb_type, idx)
	world.add_body(name, color=np.ones(4))
	pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
	return name


def round_if_float(value):
	if isinstance(value, float):
		return Decimal(str(value)).quantize(Decimal('1.00'))
	else:
		return value

def rounded_tuple(tup):
	return tuple(round_if_float(value) for value in tup)

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




# def collision(robot_config):
# 	return collision_fn(robot_config)

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
	lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
	generator = interval_generator(lower_limits, upper_limits, **kwargs)
	def fn():
		return tuple(next(generator))
	return fn

#generate steps between start and end configurations
def interpolate_configs(start_config, end_config, num_steps = int(10)):
	out = []
	start_config = list(start_config)
	end_config = list(end_config)

	#print('start config:', start_config)
	#print('end config:', end_config)

	for i in range(num_steps):
		fraction = float(i) / num_steps
		list1 = list(x * (1-fraction) for x in start_config) 
		list2 = list(x * (fraction) for x in end_config)
		config = [a + b for a, b in zip(list1, list2)]
		out.append(tuple(config))
	return out



class rrt_helpers:

	def __init__(self, joint_number, start_config, joint_limits, joint_names):
		self.joints = joint_number
		self.rrt = [start_config]
		self.parents = {} # a dictionary key: tuple(a config), value: tuple(parent's config)
		self.joint_limits = joint_limits
		self.joint_names = joint_names

	def current_pos(self):
		joint_idx = [joint_from_name(world.robot, joint) for joint in self.joint_names]
		return tuple(get_joint_positions(world.robot, joint_idx))

	#collision fuction
	def no_collision(self,start_config, end_config):
		i = 0
		for step in interpolate_configs(start_config, end_config):
			# set_joint_positions(world.robot, world.arm_joints, step)
			#wait_for_user('step' + str(i))
			#i += 1
			if single_collision(world.robot):
				return False
		return True

	def find_nearest_neighbor(self,rand_node):
		distances = []
		for node in self.rrt:
			# print("NODE",node, rand_node)
			distances.append(self.distance(node, rand_node))
		min_index = np.argmin(distances)
		return self.rrt[min_index]

	def direction(self,config1, config2):
		# return the direction of each 7 DOF joint (a list)
		# print("configs", config1, "2", config2)
		dirs = []
		for i in range(self.joints):
			if config1[i] > config2[i]:
				# print("clockwise")
				dirs.append(-1) # clockwise
			elif config1[i] <= config2[i]:
				# print("anticlock")
				dirs.append(+1) # counter-clockwise
			# else:
				# print("none")
				# dirs.append(0)
		return dirs


	def distance(self,n1, n2):
		# print("n1", n1, "n2", n2,"en(joint_limits)",self.joints)
		dist = 0
		weights = [0.1, 2.5, 0.6, 0.4, 0.3, 0.1, 1.0]
		for i in range(self.joints):
			dist += abs( n1[i] - n2[i])*weights[i]
		return dist # Sum of angle error TODO: Distance Metrics! 6DOF


	def sample_node(self,goal_node, goal_bias_prob, joints_start, joints_scale):
		# r = rd.random() # float [0.0, 1.0)
		r = np.random.random(1)
		if r <= goal_bias_prob:
			return goal_node 
		else:
			random_config = []
			rand_angles = np.random.rand(self.joints)
			for i in range(self.joints):
				# random_angle = rd.random() # TODO: uniform random
				random_config.append(joints_start[i] + rand_angles[i] * joints_scale[i])      
			
			return tuple(random_config)
	def rrt_connect(self,near_node, rand_node, step_size, curmincost, goal_node):
		dirs = self.direction(near_node,rand_node)
		# print("near_node",near_node,"dirs", dirs)
		canbreak = True # assume go to rand and can break!
		new_node = [0, 0, 0, 0, 0, 0,0]
		for i in range(self.joints):
			if abs(rand_node[i] - near_node[i]) <= step_size:
				new_node[i] = rand_node[i]
			else:
				new_node[i] = near_node[i] + dirs[i]*step_size
				canbreak = False
		if(canbreak):
			if self.distance(goal_node, new_node) < curmincost:
				curmincost = self.distance(goal_node, new_node)
				print("1 update cur min cost: ",curmincost)
			self.rrt.append(rand_node)
			self.parents[rand_node] = near_node
			return self.rrt[-1], curmincost

		while self.no_collision(self.current_pos(),new_node) and self.in_limit(new_node) and not canbreak:
		# while self.in_limit(new_node) and not canbreak:
			if self.distance(goal_node, new_node) < curmincost:
				curmincost = self.distance(goal_node, new_node)
				print("2 update cur min cost: ",curmincost)

			self.rrt.append(tuple(new_node))
			self.parents[tuple(new_node)] = tuple(near_node)
			near_node = tuple(new_node) # update parent
			
			canbreak = True
			for i in range(self.joints):
				if abs(rand_node[i] - near_node[i]) <= step_size:
					new_node[i] = rand_node[i]
					# print("joint ",i," reach rand_node")        
				else:
					new_node[i] = near_node[i] + dirs[i]*step_size
					canbreak = False

			if canbreak:
				self.rrt.append(tuple(new_node))
				self.parents[tuple(new_node)] = near_node
				break

		return self.rrt[-1], curmincost


	def step(self,near_node, rand_node, step_size):
		dirs = direction(near_node, rand_node)
		new_node = tuple(near_node[i] + dirs[i]*step_size for i in range(len(joint_limits))) # stear ONE STEP from near to rand 'direction'!
		return new_node



	def reach_goal(self,goal_node, node, goal_threshold):
		# print("Distance from cur pose to goal config: ",distance(goal_node, node))
		return self.distance(goal_node, node) <= goal_threshold

	def extract_path(self,parents, node, root_parent, debug=False):
		path = []
		while parents[node]!= root_parent:
			if debug:
				print("Node:   ",rounded_tuple(node))
				print("Parent: ",rounded_tuple(parents[node]))
			path.append(node)
			node = parents[node]
		path.reverse()
		return path


	def in_limit(self,config):
		for i in range(self.joints):
			if (config[i] < self.joint_limits[self.joint_names[i]][0] or \
			   config[i] > self.joint_limits[self.joint_names[i]][1]) and i != 4: # KEY: joint 4 has no limit!
			   print("joint ",i," out of bound.",self.joint_limits[self.joint_names[i]], " angle: ",config[i])
			   return False
		return True

# def move_robot_arm(world, target_pose):
#     '''
#     moves the robot end effector to the target_pose
#     @param world: world object
#     @param target_pose: Pose object that the end effector will move to
#     '''

#     tool_link = link_from_name(world.robot, 'panda_hand')

#     sample_fn = _get_sample_fn(world.robot, world.arm_joints)

#     conf = sample_fn()
#     set_joint_positions(world.robot, world.arm_joints, conf)
#     ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
#     start_pose = get_link_pose(world.robot, tool_link)
	
#     for pose in interpolate_poses(start_pose, target_pose, pos_step_size=0.01):
#         conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
#         if conf is None:
#             print('Failure!')
#             wait_for_user()
#             break
#         set_joint_positions(world.robot, ik_joints, conf)



def main():
	print('Random seed:', get_random_seed())
	print('Numpy seed:', get_numpy_seed())

	np.set_printoptions(precision=3, suppress=True)

	# print("worls", world)
	sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
	spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
	# wait_for_user()
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
	#specifying obstacles!
	obstacles = get_links(world.kitchen)
	obstacle_names = [get_link_name(world.kitchen, link) for link in obstacles]


	robotlinks = get_links(world.robot)
	robotlinks_names = [get_link_name(world.robot, link) for link in robotlinks]
	# print("obs" ,obstacles)
	# print("obstacles are ",robotlinks_names)

	#test = p.getClosestPoints(joint_idx[0],obstacles[1],1000000)
	tool_link = link_from_name(world.robot, 'panda_hand')

	kitchenlinks = get_links(world.kitchen)
	kitchenlinks_names = [get_link_name(world.kitchen, link) for link in kitchenlinks]
	# print("obs" ,obstacles)
	# print("KITCHEN are ",kitchenlinks_names)

	# start_config = tuple(get_joint_positions(world.robot, joint_idx))
	sample_fn = get_sample_fn(world.robot, world.arm_joints)
	start_config = sample_fn()

	# goal_config = (0.5, 0.33, -1.548, 1.557, -1.32, -0.1928, 0)
	# goal_config = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

	robot_default_pos = get_link_pose(world.robot, tool_link)
	print("START IS ",start_config, robot_default_pos)

	goal_config = get_pose(world.get_body(sugar_box))
	print("GOAL IS ",goal_config, sugar_box)
	goal_config = goal_config[0] + goal_config[1]
	print("GOAL IS ",goal_config)



	tool_link = link_from_name(world.robot, 'panda_hand')
	start_pose = get_link_pose(world.robot, tool_link)
	print("Pose of panda_hand: ", start_pose)
	print("Euler angles of panda_hand: ", euler_from_quat(start_pose[1]))

	joints_pos = get_joint_positions(world.robot, world.arm_joints)
	arm_joint_names = [get_joint_name(world.robot, joint) for joint in world.arm_joints]
	print("Position of joints: ")
	for joint, pos in zip(arm_joint_names, joints_pos):
		print("    ", joint, ":", pos)

	base_joints_pos = get_joint_positions(world.robot, world.base_joints)
	print("Base position: X -", base_joints_pos[0], 'Y -', base_joints_pos[1], 'Z -', base_joints_pos[2])


	wait_for_user()
# 
	drawer_joint = joint_from_name(world.kitchen, 'indigo_drawer_top_joint')
	print("Drawer has the following joint limits: ", get_joint_limits(world.kitchen, drawer_joint))
	drawer_pos = input("What position do you want the drawer at?")
	drawer_pos = float(drawer_pos.strip())

	set_joint_position(world.kitchen, drawer_joint, drawer_pos)


	wait_for_user('Going to set base in front of sugar box')
	point1_arm = (-0.1, 1.25, 0, -0.6, 0 ,3.04 , 0.741)
	point1_base = (0.8, 0.5, -3.14)
	set_joint_positions(world.robot, world.base_joints, point1_base)
	set_joint_positions(world.robot, world.arm_joints, point1_arm)

	wait_for_user('Grasping sugar box')
	sugar_box_att = create_attachment(world.robot, link_from_name(world.robot, 'panda_hand'), world.get_body(sugar_box))

	wait_for_user('Attempting to move sugar box')
	point2_arm = (3.14, 1.25, 0, -0.6, 0 ,3.04 , 0.741)
	set_joint_positions(world.robot, world.arm_joints, point2_arm)
	sugar_box_att.assign()

	wait_for_user('Attempting to move sugar box')
	point2_arm = (0, 1.25, 0, -0.6, 0 ,3.04 , 0.741)
	set_joint_positions(world.robot, world.arm_joints, point2_arm)
	sugar_box_att.assign()


	object_dict = {
		'potted_meat_can1': (0.7, 0.3, -np.pi/2),
		'sugar_box0': (0.75, 0.65 , -np.pi/2)
	}

	user_obj = input("What object would you like to locate? ")
	user_obj = user_obj.strip()

	try:
		link = link_from_name(world.kitchen, user_obj)
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







	set_joint_positions(world.robot, world.base_joints, (0.7, 0.6, -np.pi/2))

	user_choice = input("Would you like the drawer opened or closed?")
	user_choice = user_choice.strip()

	target_pose = get_link_pose(world.kitchen, link_from_name(world.kitchen, 'indigo_drawer_handle_top'))

	target_pos, target_quat = target_pose
	tool_target_pos = ((target_pos[0]+0.1), target_pos[1], target_pos[2])
	orig_target_euler = euler_from_quat(target_quat)

	tool_target_euler = (-orig_target_euler[0], orig_target_euler[1], (0.5*np.pi))
	tool_target_quat = quat_from_euler(tool_target_euler)

	tool_link = link_from_name(world.robot, 'panda_hand')

	print("Attempting to get joint angles for target pose: ", target_pos, tool_target_euler)

	target_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, (tool_target_pos, tool_target_quat), max_time=0.05), None)

	if target_joint_angles == None:
		print("Unable to get target joint angles!")

	else:
		print("Setting arm to target joint angles!")
		set_joint_positions(world.robot, world.arm_joints, target_joint_angles)

		target_x, target_y, target_z = tool_target_pos
		
		if user_choice.lower() == 'opened':
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

			if user_choice.lower() == 'opened':
				drawer_pos = 0.4
			else:
				drawer_pos = 0

			set_joint_position(world.kitchen, joint_from_name(world.kitchen, 'indigo_drawer_top_joint'), drawer_pos)









	print("Going to operate the base without collision checking")
	for i in range(100):
		goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
		set_joint_positions(world.robot, world.base_joints, goal_pos)
		# if (i % 30 == 0):
		# 	wait_for_user()
	#base spawn position
	spawn_pos = np.array([.85, .8, np.pi])
	set_joint_positions(world.robot, world.base_joints, spawn_pos)


	tool_link = link_from_name(world.robot, 'panda_hand')
	start_pose = get_link_pose(world.robot, tool_link)
	print("Pose of panda_hand: ", start_pose)
	print("Euler angles of panda_hand: ", euler_from_quat(start_pose[1]))

	joints_pos = get_joint_positions(world.robot, world.arm_joints)
	arm_joint_names = [get_joint_name(world.robot, joint) for joint in world.arm_joints]
	print("Position of joints: ")
	for joint, pos in zip(arm_joint_names, joints_pos):
		print("    ", joint, ":", pos)

	base_joints_pos = get_joint_positions(world.robot, world.base_joints)
	print("Base position: X -", base_joints_pos[0], 'Y -', base_joints_pos[1], 'Z -', base_joints_pos[2])


	wait_for_user()

	path = []
	rrt_obj = rrt_helpers(len(joint_limits),start_config, joint_limits, joint_names)

	# collision_fn = get_collision_fn_franka(world.robot, joint_idx, list(obstacles))
	# collision_fn = rrt_obj.no_collision

	# Example use of collision checking
	# print("Robot colliding? ", no_collision((0.5, 1.19, -1.548, 1.557, -1.32, -0.1928)))
	print("Robot not colliding? ", rrt_obj.no_collision(start_config,goal_config))
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
	goal_bias_prob = 0.2 # goal_bias: 10%
	goal_node = goal_config
	root_parent = (-1,-1,-1,-1,-1,-1)
	# Total: 6 DOF 
	K = 3000  # 10000 nodes iter: 81, rrt# 987
	# rrt = [start_config]     # a list of config_nodes
	rrt_obj.parents[start_config] = root_parent

	# goal_threshold = 0.5
	goal_threshold = 0.1
	findpath = False



	curmincost = rrt_obj.distance(start_config,goal_config)


	print("=================================")
	print("Start config: ",start_config)
	print("Goal config: ",goal_config)
	print("Starting Error: ",rrt_obj.distance(start_config,goal_config))
	print("RRT algorithm start: ...\n")

	FACTOR = 50
	final_node = (-1, -1, -1, -1, -1, -1) # super important
	################ RRT Algorithm here #####################
	for i in range(K): # from 0 to K-1
		if i% FACTOR ==0:
			print("iter ",i)
		rand_node = rrt_obj.sample_node(goal_node, goal_bias_prob, joints_start, joints_scale) # (with 0.1 goal bias)
		# print("RAND", rand_node)

		nearest_node = rrt_obj.find_nearest_neighbor(rand_node)
		new_node, curmincost = rrt_obj.rrt_connect(nearest_node, rand_node, step_size, curmincost, goal_node)
		# print("iter", i)
		if rrt_obj.reach_goal(goal_node, new_node, goal_threshold):
			findpath = True
			print("Reach goal! At iter ",i," # nodes in rrt: ", len(rrt_obj.rrt))
			print("Final pose: ",new_node)
			final_node = new_node
			path = rrt_obj.extract_path(rrt_obj.parents, new_node, root_parent)
			break


	if not findpath:
		print('Failure!')
#             wait_for_user()
#             break
		print("No path found under current configuratoin")

	print("Now executing first found path: ")    

	print("Draw left end-effector position (red sphere):")
	radius = 0.03
	color = (1, 0, 0, 1)
	for pose in path:
		set_joint_positions(world.robot, joint_idx, pose)
		ee_pose = get_link_pose(world.robot, link_from_name(world.robot, 'right_gripper'))
		draw_sphere_marker(ee_pose[0], radius, color)

	print("Planner run time: ", time.time() - start_time)


	goal_config = get_pose(world.get_body(sugar_box))
	print("GOAL IS ",goal_config, sugar_box)
	goal_config = goal_config[0] + goal_config[1]
	print("GOAL IS ",goal_config)
	sugar_init = goal_config
	rand_node = rrt_obj.sample_node(sugar_init, goal_bias_prob, joints_start, joints_scale) # (with 0.1 goal bias)

	rrt_obj2 = rrt_helpers(len(joint_limits),rand_node, joint_limits, joint_names)

	# body = body_from_name(kitchen_part_right)
	# return get_link_pose(body, link_from_name(body, indigo_countertop))
	# drop_sugar= Pose(get_link_pose(world.kitchen, link_from_name(world.kitchen, "indigo_countertop"))[0][:2] + [-0.25]),  # Pose of the arm before dropping the sugar
	drop_sugar= get_link_pose(world.kitchen, link_from_name(world.kitchen, "indigo_countertop")),  # Pose of the arm before dropping the sugar
	drop_sugar = drop_sugar[0][0] + drop_sugar[0][1]
	print("drop sugar", drop_sugar, type(drop_sugar[0]))
	print(" sugar", sugar_init, type(sugar_init))
	# print("drop sugar", len(drop_sugar[0]), drop_sugar[0][0])
	rrt_obj2.parents[sugar_init] = root_parent
	goal_node = drop_sugar
	# goal_threshold = 0.5
	goal_threshold = 0.4
	findpath = False



	curmincost = rrt_obj2.distance(sugar_init,drop_sugar)


	print("=================================")
	print("Start config: ",sugar_init)
	print("Goal config: ",drop_sugar)
	print("Starting Error: ",rrt_obj2.distance(rand_node,drop_sugar))
	print("RRT algorithm start: ...\n")

	FACTOR = 50
	final_node = (-1, -1, -1, -1, -1, -1) # super important
	################ RRT Algorithm here #####################
	for i in range(K): # from 0 to K-1
		if i% FACTOR ==0:
			print("iter ",i)
		rand_node = rrt_obj2.sample_node(drop_sugar, goal_bias_prob, joints_start, joints_scale) # (with 0.1 goal bias)
		# print("rand", rand_node)
		nearest_node = rrt_obj2.find_nearest_neighbor(rand_node)
		new_node, curmincost = rrt_obj2.rrt_connect(nearest_node, rand_node, step_size, curmincost, drop_sugar)

		if rrt_obj2.reach_goal(drop_sugar, new_node, goal_threshold):
			findpath = True
			print("Reach goal! At iter ",i," # nodes in rrt: ", len(rrt_obj2.rrt))
			print("Final pose: ",new_node)
			final_node = new_node
			path = rrt_obj2.extract_path(rrt_obj2.parents, new_node, root_parent)
			break

	if not findpath:
		print("No path found under current configuratoin")

	print("Now executing first found path: ")    

	print("Draw left end-effector position (red sphere):")
	radius = 0.03
	color = (1, 0, 0, 1)
	for pose in path:
		set_joint_positions(world.robot, joint_idx, pose)
		ee_pose = get_link_pose(world.robot, link_from_name(world.robot, 'right_gripper'))
		draw_sphere_marker(ee_pose[0], radius, color)

	print("Planner run time: ", time.time() - start_time)
	# print("Run Shortcut Smoothing Algorithm for 150 iterations...")

	# def try_shortcut(parents, node1, node2, step_size):
	# 	dirs = rrt_obj.direction(node1,node2)

	# 	new_nodes = []
	# 	tmp_parents = {}

	# 	canbreak = True # assume go to rand and can break!
	# 	new_node = [0, 0, 0, 0, 0, 0,0]
	# 	near_node = deepcopy(node1)

					
	# 	for i in range(len(joint_limits)):
	# 		if abs(node2[i] - near_node[i]) <= step_size:
	# 			new_node[i] = deepcopy(node2[i])
	# 		else:
	# 			new_node[i] = near_node[i] + dirs[i]*step_size
	# 			canbreak = False
	# 	# near_node2 = [0, 0, 0, 0, 0, 0]
	# 	if(canbreak):
	# 		tmp_parents[tuple(node2)] = node1
	# 		for key in tmp_parents:
	# 			if key == tmp_parents[key] :
	# 				print("node 1 to node 2")
	# 				print("key: ", rounded_tuple(key))
	# 				print("val : ",rounded_tuple(tmp_parents[key]))
	# 				print("Deadlock here!")

	# 			parents[key] = tmp_parents[key] # copy and paste only if success!
	# 		return True, new_nodes, parents

	# 	near_node2 = deepcopy(near_node)
	# 	while rrt_obj.no_collision(rrt_obj.current_pos(),new_node) and rrt_obj.in_limit(new_node) and not canbreak:
	# 		new_nodes.append(tuple(new_node))
	# 		tmp_parents[tuple(new_node)] = tuple(near_node)
	# 		if new_node == near_node:
	# 			print("Deadlock due to here!")
	# 		# parents[tuple(new_node)] = tuple(near_node)
	# 		near_node = deepcopy(tuple(new_node)) # update parent
	# 		canbreak = True
	# 		# update new_node
	# 		for i in range(len(joint_limits)):
	# 			if abs(node2[i] - near_node[i]) <= step_size:
	# 				new_node[i] = node2[i]       

	# 			else:
	# 				new_node[i] = near_node[i] + dirs[i]*step_size
	# 				canbreak = False
	# 		near_node2 = deepcopy(near_node) ## KEY!! Why need another variable? use coy!

	# 	# Problem here: Deadlock on the 'last node' : set parent[node2] to be near
	# 	if canbreak:
	# 		new_nodes.append(tuple(node2))
	# 		# print("len of tmp parents: ",len(tmp_parents))
	# 		lastkey = node1
	# 		for key in tmp_parents:
	# 			# print("key: ", rounded_tuple(key))
	# 			# print("val : ",rounded_tuple(tmp_parents[key]))
	# 			if key == tmp_parents[key] :
	# 				tmp_parents[key] = lastkey
	# 			# Successful shortcut! Add tmp_parents {key:value} to parents
	# 			rrt_obj.parents[key] = tmp_parents[key] # copy and paste only if success!
	# 			lastkey = deepcopy(key)
	# 		return True, new_nodes, rrt_obj.parents
	# 	else:
	# 		# Fail, don't do any modification
	# 		return False, new_nodes, rrt_obj.parents


	# ITERATIONS = 150 # should be 150
	# for it in range(ITERATIONS):
	# 	id1, id2 = np.random.randint(low=0, high =len(path)-1, size=2)
	# 	while(id1 == id2): # KEY : CANNOT Pick two equal points! Otherwise: Deadlock!!
	# 		id1, id2 = np.random.randint(low=1, high=len(path)-2, size=2)
	# 	if id1 < id2:
	# 		n1 = path[id1]
	# 		n2 = path[id2]
	# 	else:
	# 		n1 = path[id2]
	# 		n2 = path[id1]
	# 	success, new_nodes, rrt_obj.parents =  try_shortcut(rrt_obj.parents, n1, n2, step_size)

	# print("Extracing optimized path...")
	# optimized_path = rrt_obj.extract_path(rrt_obj.parents, final_node, root_parent, False) #debug=True)

	# print("\n =======================")
	# print("First Path    : ", len(path)," nodes")
	# print("Optimized Path: ", len(optimized_path), " nodes")


	# print("Now execute optimized_path:")

	# print("Optimized path:")
	# bluecolor = (0, 0, 1, 1)
	# for pose in optimized_path:
	# 	print(rounded_tuple(pose))
	# 	set_joint_positions(world.robot, joint_idx, pose)
	# 	ee_pose = get_link_pose(world.robot, link_from_name(world.robot, 'right_gripper'))
	# 	draw_sphere_marker(ee_pose[0], radius, bluecolor)
	# ######################
	
	# # Execute planned path
	# print("Now executing shortcut smoothed path: ")    
	# execute_trajectory(world.robot, joint_idx, optimized_path, sleep=0.1)



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

