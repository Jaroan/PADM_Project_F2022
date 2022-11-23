import gitmodules
__import__('padm-project-2022f') 

import pybullet as p

from src.world import World
from pybullet_tools.utils import joint_from_name, get_joint_info, get_links, get_link_name, get_joint_positions, wait_for_user, \
    set_joint_positions 

from src.utils import translate_linearly


world = World(use_gui=True)

#specifying joint limits!
joint_names =('panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7')
joint_idx = [joint_from_name(world.robot, joint) for joint in joint_names]
joint_limits = {joint_names[i] : (get_joint_info(world.robot, joint_idx[i]).jointLowerLimit, get_joint_info(world.robot, joint_idx[i]).jointUpperLimit) for i in range(len(joint_idx))}
#print("joint_limits:", joint_limits)

#specifying obstacles!
obstacles = get_links(world.kitchen)
obstacle_names = [get_link_name(world.kitchen, link) for link in obstacles]
#print(obstacle_names)

#test = p.getClosestPoints(joint_idx[0],obstacles[1],1000000)

goal_pos = translate_linearly(world, 2) # does not do any collision checking!!
set_joint_positions(world.robot, world.base_joints, goal_pos)


test = p.getContactPoints(bodyA = 1, bodyB = 2)
wait_for_user()

print('contact points:', test, 'length:', len(test))

wait_for_user()
p.disconnect()