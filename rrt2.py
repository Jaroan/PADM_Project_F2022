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

from pybullet_tools.utils import connect, disconnect, draw_circle,  get_movable_joint_descendants, wait_for_user, wait_if_gui, get_joint_name, \
	 joint_from_name, get_joint_positions, set_joint_positions, get_joint_info, get_link_pose, link_from_name, get_joint_limits,\
	  body_from_name, get_euler, get_links, get_link_name,  single_collision, get_custom_limits, interval_generator, get_movable_joints





# world = World(use_gui=True)




def round_if_float(value):
	if isinstance(value, float):
		return Decimal(str(value)).quantize(Decimal('1.00'))
	else:
		return value

def rounded_tuple(tup):
	return tuple(round_if_float(value) for value in tup)


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

	def __init__(self, world,  start_config):

		
		self.joint_names =('panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7')
		# joint_names = (1,2,3,4,5,6,7)
		self.joint_idx = [joint_from_name(world.robot, joint) for joint in self.joint_names]
		self.world = world
		# parse active DoF joint limits
		self.joint_limits = {self.joint_names[i] : (get_joint_info(world.robot, self.joint_idx[i]).jointLowerLimit, get_joint_info(world.robot, self.joint_idx[i]).jointUpperLimit) for i in range(len(self.joint_idx))}
		self.joints_start = [self.joint_limits[self.joint_names[i]][0] for i in range(len(self.joint_limits))]
		self.joints_scale = [abs(self.joint_limits[self.joint_names[i]][0] - self.joint_limits[self.joint_names[i]][1]) for i in range(len(self.joint_limits))]

		self.joints = len(self.joint_limits)
		self.rrt = [start_config]
		self.parents = {} # a dictionary key: tuple(a config), value: tuple(parent's config)
		# self.joint_limits = joint_limits
		# self.joint_names = joint_names

	def current_pos(self):
		joint_idx = [joint_from_name(self.world.robot, joint) for joint in self.joint_names]
		return tuple(get_joint_positions(self.world.robot, joint_idx))

	#collision fuction
	def no_collision(self,start_config, end_config):
		i = 0
		for step in interpolate_configs(start_config, end_config):
			# set_joint_positions(world.robot, world.arm_joints, step)
			#wait_for_user('step' + str(i))
			#i += 1
			if single_collision(self.world.robot):
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


	def sample_node(self,goal_node, goal_bias_prob):
		# r = rd.random() # float [0.0, 1.0)
		r = np.random.random(1)
		if r <= goal_bias_prob:
			return goal_node 
		else:
			random_config = []
			rand_angles = np.random.rand(self.joints)
			for i in range(self.joints):
				# random_angle = rd.random() # TODO: uniform random
				random_config.append(self.joints_start[i] + rand_angles[i] * self.joints_scale[i])      
			
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
		print("check")
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
