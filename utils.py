import gitmodules # need to pip install
__import__('padm-project-2022f') 
__import__('ss-pybullet') 
# from pybullet_tools.utils  import get_collision_fn, get_floating_body_collision_fn, expand_links
from pybullet_tools.utils import CIRCULAR_LIMITS,set_joint_positions, \
    wait_if_gui, wait_for_duration, interval_generator, get_custom_limits, get_collision_data,BASE_LINK
import pybullet as p


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

def get_collision_fn_franka(robot, joints, obstacles):
    # check robot collision with environment
    # return get_collision_fn(robot, joints, obstacles=obstacles, attachments=[], \
    #     self_collisions=True)
    return get_collision_data(robot, joints)

def execute_trajectory(robot, joints, path, sleep=None):
    # Move the robot according to a given path
    if path is None:
        print('Path is empty')
        return
    print('Executing trajectory')
    for bq in path:
        set_joint_positions(robot, joints, bq)
        if sleep is None:
            wait_if_gui('Continue?')
        else:
            wait_for_duration(sleep)
    print('Finished')


def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return 
   
def execute_trajectory(robot, joints, path, sleep=None):
    # Move the robot according to a given path
    if path is None:
        print('Path is empty')
        return
    print('Executing trajectory')
    for bq in path:
        set_joint_positions(robot, joints, bq)
        if sleep is None:
            wait_if_gui('Continue?')
        else:
            wait_for_duration(sleep)
    print('Finished')


##################################
################################################################################

def custom_limits_from_base_limits(robot, base_limits, yaw_limit=None):
    x_limits, y_limits = zip(*base_limits)
    custom_limits = {
        joint_from_name(robot, 'x'): x_limits,
        joint_from_name(robot, 'y'): y_limits,
    }
    if yaw_limit is not None:
        custom_limits.update({
            joint_from_name(robot, 'theta'): yaw_limit,
        })
    return custom_limits

def get_descendant_obstacles(body, link=BASE_LINK):
    # TODO: deprecate?
    return {(body, frozenset([link]))
            for link in get_link_subtree(body, link)}

################################################################################

Z_EPSILON = 2.5e-3


# def open_surface_joints(world, surface_name, joint_names=ALL_JOINTS):
#     surface = surface_from_name(surface_name)
#     for joint_name in surface.joints:
#         joint = joint_from_name(world.kitchen, joint_name)
#         if joint_name in joint_names:
#             # TODO: remove this mechanic in the future
#             world.open_door(joint)

def get_surface_obstacles(world, surface_name):
    surface = surface_from_name(surface_name)
    obstacles = set()
    for joint_name in surface.joints:
        link = child_link_from_joint(joint_from_name(world.kitchen, joint_name))
        obstacles.update(get_descendant_obstacles(world.kitchen, link))
    # Be careful to call this before each check
    open_surface_joints(world, surface_name, joint_names=CABINET_JOINTS)
    return obstacles

def test_supported(world, body, surface_name, collisions=True):
    # TODO: is_center_on_aabb or is_placed_on_aabb
    surface_aabb = compute_surface_aabb(world, surface_name)
    # TODO: epsilon thresholds?
    if not is_placed_on_aabb(body, surface_aabb):  # , above_epsilon=z_offset+1e-3):
        return False
    obstacles = world.static_obstacles | get_surface_obstacles(world, surface_name)
    if not collisions:
        obstacles = set()
    #print([get_link_name(obst[0], list(obst[1])[0]) for obst in obstacles
    #       if pairwise_collision(body, obst)])
    return not any(pairwise_collision(body, obst) for obst in obstacles)


def get_link_obstacles(world, link_name):
    if link_name in world.movable:
        return flatten_links(world.get_body(link_name))
    elif has_link(world.kitchen, link_name):
        link = link_from_name(world.kitchen, link_name)
        return flatten_links(world.kitchen, get_link_subtree(world.kitchen, link)) # subtree?
    assert link_name in SURFACE_FROM_NAME
    return set()

################################################################################

def are_confs_close(conf1, conf2, tol=1e-8):
    assert (conf1.body == conf2.body) and (conf1.joints == conf2.joints)
    difference_fn = get_difference_fn(conf1.body, conf1.joints)
    difference = difference_fn(conf1.values, conf2.values)
    return np.allclose(difference, np.zeros(len(conf1.joints)), rtol=0., atol=tol)


def translate_linearly(world, distance):
    # TODO: could just apply in the base frame
    x, y, theta = get_joint_positions(world.robot, world.base_joints)
    pos = np.array([x, y])
    goal_pos = pos + distance * unit_from_theta(theta)
    goal_pose = np.append(goal_pos, [theta])
    return goal_pose
