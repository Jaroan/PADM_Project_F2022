import gitmodules
__import__('padm-project-2022f') 

import pybullet as p

from src.world import World
from pybullet_tools.utils import joint_from_name, get_joint_info, get_links, get_link_name, get_joint_positions, wait_for_user, set_joint_positions, single_collision, \
    get_custom_limits, CIRCULAR_LIMITS, interval_generator

from src.utils import translate_linearly

# world = World(use_gui=True)

# #specifying joint limits!
# joint_names =('panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7')
# joint_idx = [joint_from_name(world.robot, joint) for joint in joint_names]
# joint_limits = {joint_names[i] : (get_joint_info(world.robot, joint_idx[i]).jointLowerLimit, get_joint_info(world.robot, joint_idx[i]).jointUpperLimit) for i in range(len(joint_idx))}
# #print("joint_limits:", joint_limits)

# wait_for_user()

# #specifying obstacles!
# obstacles = get_links(world.kitchen)
# obstacle_names = [get_link_name(world.kitchen, link) for link in obstacles]

#generate position based on joint limits
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

#collision fuction
def no_collision(start_config, end_config):
    i = 0
    for step in interpolate_configs(start_config, end_config):
        set_joint_positions(world.robot, world.arm_joints, step)
        #wait_for_user('step' + str(i))
        #i += 1
        if single_collision(world.robot):
            return False
    return True

#generate random config for start and end positions
# sample_fn = get_sample_fn(world.robot, world.arm_joints)
# random_config1 = sample_fn()
# random_config2 = sample_fn()

# print('no collisions?', no_collision(random_config1, random_config2) )


# wait_for_user()
# p.disconnect()