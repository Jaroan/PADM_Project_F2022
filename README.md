# 16.410 PADM Project Fall 2022

<!-------------------------------------------------------------------------->
<i>Jasmine Jerry Aloor and Alissa Chavalithumrong</i>

## Introduction

#### <b> Learning Objectives </b>
- Apply search, planning, constraint satisfaction, optimization, and probabilistic 
reasoning algorithms to real-world autonomy and decision-making problems
- Implement  search,  planning,  constraint  satisfaction, and optimization.
- Learn about problem formulations using .pddl
- Implement pybullet and pydrake libaries

<!-------------------------------------------------------------------------->

## Section 1: Activity Planning

#### <b> Assumptions Made </b>
1. Every item location is known and can be reached by the robot.

#### <b> Plan Generation </b>

 We design our `object` type to be made up of a `location` and a `locatable`. From there, we break down our `locations` to be `top`,  and `drawer`. Similarly, our `locatables` are `food` and `bot`. This is illustrated in <i>Figure 1</i>.

 In our problem.pddl file, through typing, we are able to specify our object. Spam and sugar are listed as a `food` type. The gripper is a special `locatable` that will be frequently queried, we use `robot` to further classify it. Our locations are broken down as drawer as `drawer` and countertop and stovetop as `top`. This is due to specific contraints that are required when placing items on a top vs drawer locations.

In our domain file, we have 4 predicates

- `(gripper-empty)`: is the gripper empty? 
- `(gripper-holding ?object - locatable)`: what object is the gripper holding?
- `(on ?location - location ?object - locatable)`: what location is an object? 
- `(drawer-open)`: is the drawer open?

<p align="center">
 <img src="heirarchy_domain.png" alt="heirarchy" width="400"/>
 </p>

 <p align="center"> <i>Figure 1:</i> Domain heirarchy  </p>

#### <b> File Structure </b>

- `domain.pddl`: defines the “universal” aspects of the kitchen problem
- `problem.pddl`: defines the global “worldly” aspects of the kitchen problem
- `planner.py`: takes .pddl files and uses breath-first search to calculate a plan

#### <b> Challenges </b>
- In our first iteration of the .pddl files, we struggled with generating a valid domain and problem file that was solvable by our planner. 

<!-------------------------------------------------------------------------->

## Section 2: Sample Based Motion Planning

#### <b> Assumptions Made </b>
1. All locatables (spam box and sugar box) and locations (countertop, drawer, stovetop) goal positions are known to the robot/motion planner. 
2. Because the robotic gripper can reach all desired goal positions from one location near the kitchen, our base spawn location will be at that fixed position. Our spawn arm position will initalize in a neutral position where there are no possible collisions.

#### <b> RRT Motion Planner </b>
The motion planner that was implimented was RRT. We used RRT to calculate a path from our initial position to a fixed goal position while avoiding collisions.
We generated an activity plan, and from there parsed the actions and parameters. 

For example, in the  activity step `move arm counter drawer`, we would move the `arm` from it's current position at the `counter` to the `drawer`. 
We then iterate through each step in of the activity plan and have the robot perform each action. 
In the case of picking up, dropping, and opening/closing the drawer, we hard-coded each item to move with the gripper. 

#### <b> Results</b>
- insert GIF of RRT planner here!!

<!-- 
<p align="center">
 <img src="name_here.gif" alt="optimized_traj" width="500"/>
 </p>
-->

#### <b> File Structure </b>

- `collision.py`: contains collision checking functions for rrt
- `rrt.py`
- `utils.py`: helper functions for rrt
- `project_run.py`
- 🤪

#### <b> Challenges</b>

This was the section that challenged us the most. Our unfamiliarity with pybullet, getting a linux virtual machine to run our arm-based macs, and initial problem formulation made the inital startup for this section very time consuming. 

<!-------------------------------------------------------------------------->

## Section 3: Trajectory Optimization

#### <b> Optimization Problem </b>

In our optimization problem, we decided to minimize distance travelled per joint. By optimizing for distance, we are able to create the shortest and smoothest path. We seed our optimization problem with a path generated by RRT, and then use the general solver `pydrake.Solve` to calculate a solution. 

$
\displaystyle{
    \begin{aligned} 
        \min_{x} &\qquad |X_{0:n-1} - X_{1:n}|^2        \\
        \text{subject to} &\qquad X_0 = \text{starting pose} \\
        &\qquad X_n = \text{goal pose} \\
        &\qquad L_{lower} \le X_i \le L_{upper}  &\qquad i = 0, 1, 2, ...  \: n\\
        &\qquad X_{i+1} - X_i \le T &\qquad i = 0, 1, 2, ...  \: n
         \\
    \end{aligned}
}
$

Our constraint optimization problem is formalized above, where $X$ is a $ n \times 7 $ matrix representing our $7$ joint positions over $n$ time-steps. 
The cost function calculates the distance travelled by every joint at each timestep. 
Our constraints are as follows: 
time-step (row) $0$ and $n$ must be equal to our starting and goal poses, 
joint positions at any time step cannot exceed lower ($L_{lower}$) and upper ($L_{upper}$) joint limits,
and joint distance at any timestep cannot exceed $T$ (a vector representing max distance each joint is able to travel). 

#### <b> Results</b>

<p align="center">
 <img src="optimized_trajectory.gif" alt="optimized_traj" width="500"/>
 </p>

Here is our resulting optimized trajectory. The red dots represent the trajectory generated by our RRT planner. The blue dots represents the new optimized trajectory.

#### <b> File Structure</b>

Our trajectory optimization code is contained within `optimization.py`.

It consists of 3 main functions:

- `trajectory_file_to_nparray`: converts trajectory_path.txt into a numpy array
- `optimization`: runs the pydrake optimazation solver and produces the new optimized trajectory (optimized_trajectory_path.npy)
- `run_optimization_simulation`: runs simulation in pybullet

#### <b> Challenges</b>
An issue that we ran into as formulating the optimization problem so that it was solvable by pydrake. IPopt and SNopt had issues solving matricies over 10 rows, which was difficult to use when our RRT code would produce trajectories of over 100 positions. We switched to using the general `pydrake.Solve` function, which allowed for pydrake to automatically select a solver that worked for our formulation. 

##  Conclusion
Although our solution is not the most robust, due to several aspects of the arm movements that were 'hard coded' (such as goal arm positioning for objects and drawer movements), we were able to sucessfully compelete every section of the project. 

Our group worked well together. We were able to creatively come up with solutions to problems and contribute equally on all sections. ❤️‍🔥 
