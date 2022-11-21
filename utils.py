from pybullet_tools.utils import set_joint_positions, \
    wait_if_gui, wait_for_duration, get_collision_fn
import pybullet as p


def get_collision_fn_franka(robot, joints, obstacles):
    # check robot collision with environment
    return get_collision_fn(robot, joints, obstacles=obstacles, attachments=[], \
        self_collisions=True)


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
