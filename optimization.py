import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import MathematicalProgram, Solve, Variable, eq, le, ge
from pydrake.solvers import IpoptSolver

import gitmodules
__import__('padm-project-2022f') 

from pybullet_tools.utils import joint_from_name, get_joint_info, set_joint_positions, get_link_pose, link_from_name, wait_for_user
from utils import  draw_sphere_marker
from src.world import World

'''convert trajectory txt file to a list of np.arrays'''
def trajectory_file_to_nparray(file_name):
    data = open(file_name)
    data_read = data.read()
    data_modified = data_read.split("\n")

    final_data = []

    for i in range(len(data_modified)):
        value = np.array(list(map(float, data_modified[i].split(', '))))
        final_data.append(value)

    return final_data

'''pydrake optimization function'''
def optimize(trajectories):

    trajectory_stack = np.hstack(trajectories)
    trajectory_length = len(trajectories)

    #initializing solution
    v = np.empty([trajectory_length, 7], dtype = Variable)
    prog = MathematicalProgram()

    for i in range(trajectory_length):
        v[i, :] = prog.NewContinuousVariables(7, 'v' + str(i))

    #creating cost function
    weights = np.array([0.5, 2.5, 0.5, 0.5, 0.5, 0.5, 1.0]) #creating weights for each arm movement [0.1, 2.5, 0.6, 0.4, 0.3, 0.1, 1.0]
    cost = np.sum( ((v[1:, :] -  v[:-1, :]) ** 2) * weights ) #total distance traveled by each joint
    prog.AddCost(cost)

    #seeding initial guess
    prog.SetInitialGuessForAllVariables(trajectory_stack)

    #maximum joint delta per time-step
    delta = 0.05
    for i in range(trajectory_length - 1):
        v1 = v[i, :]
        v2 = v[i+1, :]
        prog.AddConstraint(ge((v2 - v1), (-delta * np.ones(7)))) #lower bound
        prog.AddConstraint(le((v2 - v1), (delta * np.ones(7)))) #upper bound

    #joint limits constraints
    joint_limits = [(-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973), (-3.0718, -0.0698), (-2.8973, 2.8973), (-0.0175, 3.7525), (-2.8973, 2.8973)] 
    for i in range(7):
        prog.AddBoundingBoxConstraint(joint_limits[i][0], joint_limits[i][1],v[:, i])

    #setting intial and final positions trajectories
    v_initial = v[0, :]
    v_final = v[-1, :]
    prog.AddConstraint(eq(v_initial, trajectories[0]))
    prog.AddConstraint(eq(v_final, trajectories[-1])) 

    #couldn't get ipopt solver to work for this, so I just used the general solver instead lol
    #solver = IpoptSolver()
    result = Solve(prog)

    if not (result.is_success()):
        raise ValueError("Could not find the optimal solution.")

    solution = result.GetSolution()
    v_cost = result.get_optimal_cost()

    #print('init trajectory', trajectories[0])
    #print('final trajectory', trajectories[-1])
    #print(solution)
    print('cost:', v_cost)

    solution = solution.reshape(trajectory_length, 7)
    np.save('optimized_trajectory_path.npy', solution)

def run_optimization_simulation():
    world = World(use_gui=True)

    optimized_path = np.load('optimized_trajectory_path.npy')
    optimized_path = [tuple(item) for item in optimized_path.tolist()]

    path = trajectory_file_to_nparray("trajectory_path.txt")
    path = [tuple(item) for item in path]

    radius = 0.03

    joint_names =('panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7')
    joint_idx = [joint_from_name(world.robot, joint) for joint in joint_names]

    wait_for_user()

    color = (1, 0, 0, 1) #red
    for pose in path:
        set_joint_positions(world.robot, joint_idx, pose)
        ee_pose = get_link_pose(world.robot, link_from_name(world.robot, 'right_gripper'))
        draw_sphere_marker(ee_pose[0], radius, color)
        print(pose)

    color = (0, 0, 1, 1) #blue
    for pose in optimized_path:
        set_joint_positions(world.robot, joint_idx, pose)
        ee_pose = get_link_pose(world.robot, link_from_name(world.robot, 'right_gripper'))
        draw_sphere_marker(ee_pose[0], radius, color)
        print(pose)

    wait_for_user()

run_optimization_simulation()