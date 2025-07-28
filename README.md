# Path-Planning-for-Differential-Wheeled-Robot


## Overview
This repository provides an implementation of three path planning algorithms — A*, RRT and Potential Field — integrated within ROS2 and the Gazebo simulator. These algorithms have been specifically adapted to respect the non-holonomic constraints of a differential wheeled mobile robot (DMR). Traditional versions of these algorithms assume holonomic robots capable of moving in any direction, which is not applicable to DMRs.

To ensure the generated trajectories are feasible and executable by the robot, we implemented a path smoothing and control module using spline functions combined with kinematics-based feedback control.

The repository is designed with modularity in mind, making it easy to:

- Add new path planning algorithms.

- Integrate different control strategies.
## Project structure
- my_robot_description package: this package contains the URDF files that define the robot.
- my_robot_bringup package: this package contains the launch files used in this project. There are three launch files:
  
  1- display.launch.py/.xml: to display the robot using Rviz and check the tfs of the robot.
  
  2- my_robot_gazebo.launch.xml: to view the robot in the Gazebo simulator, where you can move it and manipulate the arm using plugins.
  
  3- robot_navigation.launch.py: this launch file is responsible for spawning the robot in gazbo simulator, read the map, plane the path based on the choosen algorithm, smooth the path and finally apply the control command to insure path tracking.
  
- my_robot_control package: this package contains the control algorithm (kinematics-based feedback) of the robot in addition to the path planning algorithm (potential field, A*).

## Installation constructions:
1. Create a workspace and inside the workspace directory Clone this repository.

```
mkdir ~/path_planning_proj
cd ~/path_planning_proj
git clone https://github.com/Zaid-NA3120/Path-Planning-for-Differential-Wheeled-robot.git
```
2. Build the workspace using Colcon build in your main directory.

```
cd ~/path_planning_proj
colcon build 
```
3. Source the workspace installation file.
```
source install/setup.bash
```
## Running the project
1. To display the robot on Rviz and check the tfs:
   ```
   ros2 launch my_robot_bringup display.launch.xml
   ```
2. To make the robot reach a desired goal:

```   
ros2 launch my_robot_bringup robot_navigation.launch.py goal_x:=4.0 goal_y:=2.0
```
## System Design and Algorithms

### Path Planning Algorithms

#### A*

The A* pathfinding algorithm is an informed, best-first search algorithm that efficiently finds the least-cost path from a start node to a goal node. It operates by maintaining a priority queue (often implemented as a min-heap) of paths, extending them one edge at a time based on a cost evaluation function:

$$ 
f(n)=g(n)+h(n)
$$

Here:

$g(n)$ is the exact cost from the start node to the current node $n$, $h(n)$ is a heuristic estimate of the cost from $n$ to the goal.

The choice of the heuristic function $h(n)$ is critical. It must be admissible, meaning it never overestimates the true cost to the goal. Under this condition, A* is guaranteed to find an optimal (least-cost) path.

In the context of a differential drive robotic model, we employ an 8-connectivity grid map to represent the robot's environment. The robot's movement is discretized into four primitive actions:
- Move forward
- Move backward
- Turn left (in place)
- Turn right (in place)

Motion along the primary axes and 45-degree turns are assigned a base cost of 5. Diagonal motions, which are more computationally expensive, incur a cost of 7, approximating $5 \times \sqrt{2}$ These costs help A* prioritize smoother and more feasible paths for the robot.

![](https://github.com/Zaid-NA3120/Path-Planning-for-Differential-Wheeled-robot/blob/main/astar_motion_gif.gif)

#### RRT (Rapidly-exploring Random Tree)

RRT is a sampling-based path planning algorithm designed for efficiently searching nonconvex, high-dimensional spaces. It is particularly effective in scenarios with complex constraints, such as robot motion planning in dynamic or cluttered environments.

RRT incrementally builds a tree rooted at the start configuration by randomly sampling the search space. The algorithm attempts to grow the tree toward each sampled point by extending the nearest existing node in the tree in the direction of the sample, subject to motion constraints and obstacle avoidance.
The basic RRT algorithm follows these steps:
1. Sample a random configuration in the space.
2. Find the nearest node in the existing tree to this sample.
3. Extend the tree by moving a fixed step size from the nearest node toward the sample.
4. Add the new node to the tree if the motion is collision-free.
5. Repeat until the goal is reached or a termination condition is met.

RRT is probabilistically complete, meaning that as the number of samples increases, the probability of finding a path (if one exists) approaches 1. However, it does not guarantee the shortest path. Variants such as RRT* improve upon this by introducing path optimization, converging toward the optimal solution over time.

For differential drive robots, the extension step must adhere to kinematic constraints—ensuring the new node is reachable under the robot’s motion capabilities. This is done by discretizing robot movements when growing the tree.


#### Potential Field

The Potential Field method is a classic path planning approach inspired by physical forces. The robot is treated as a point under the influence of an artificial potential field:

- Attractive forces pull the robot toward the goal.
- Repulsive forces push it away from obstacles.

The total potential at a point is the sum of the attractive and repulsive forces.

$$
U(x,y) = U_{att}(x,y) + U_{rep}(x,y)
$$

The attractive potential:

$$
U_{att}(x,y) = \frac{1}{2} k_{att} \cdot [(x-x_g)^2 + (y-y_g)^2] 
$$

where $$k_{att}$$ is the attractive gain and $$(x_g,y_g)$$ is the goal coordinate.

The repulsive potential:

$$
U_{rep}(x,y) = \left\lbrace
\begin{array}{ll}
    \frac{1}{2} k_{rep} (\frac{1}{d(x,y)} - \frac{1}{d_0})^2 & \mbox{, if} & d(x,y) \leq d_0 \\ 
0 & \mbox{, if} & d(x,y) > d_0 
\end{array}
\right.
$$

where $$k_{rep}$$ is the repulsive gain, $$d(x,y)$$ is the distance to the nearest obstacle, and $$d_0$$ is the repulsion influence radius.

To navigate using the potential field, the robot follows the negative gradient of the field. However, because differential-drive robots are subject to non-holonomic constraints, traditional gradient is not feasable. To address this, the planner restricts the robot's movement to discrete angular turns and forward steps. At each iteration, it evaluates a set of possible actions - such as turning or moving forward - and selects the one that minimizes the cumulative potential over a short simulated rollout.

![](https://github.com/Zaid-NA3120/Path-Planning-for-Differential-Wheeled-robot/blob/main/pf_motion.gif)

### Path Smoothing and Trajectory Generating

After generating a path using the planner (A* or Potential Field), the initial trajectory typically contains many intermediate points that are redundant for a differential mobile robot (DMR) to follow, especially in straight segments.

To optimize the path and reduce unnecessary points, we implemented a point filtering method based on slope comparison. This method preserves only the turning points (key waypoints) that represent a change in direction.
#### Algorithm description

For each consecutive triplet of points $p_{i-1}, p_i, p_{i+1}$ we compute the slopes of the two line segments:
- Slope between $p_{i-1}$ and $p_{i+1}$
- Slope between $p_{i-1}$ and $p_{i}$

If the difference between the two slopes is below a threshold, then the middle point is considered to be part of a straight line and can be discarded and the procedure is repeated with the point $p_{i+1}$ as the middle point. Otherwise, the middle point is kept and the window of calculation is moved by one point.

Finally, since most advanced control approaches require the full trajectory to ensure accurate tracking, we use spline functions to construct polynomials that represent the path as a function of time. This way, we can get polynomials that represent the velocity and acceleration of the robot.

### Controller Design

A differential-drive robot follows nonlinear kinematics, due to its dependence on orientation for motion. Controlling such a system directly is tricky because changes in the control input (linear and angular velocities) don't result in direct, linear changes in its pose.
The feedback linearization method tries to:

- Feedforward: Use the desired trajectory to compute nominal control inputs $(v_d, \omega_d)$

- Feedback: Apply corrections based on tracking error to steer the robot back to the trajectory.

The main idea is to express the error in a rotated frame aligned with the robot’s heading so that the control becomes simpler and more intuitive:

The reference trajectory:

$$
v_d = \sqrt{\dot{x}_d^2 + \dot{y}_d^2}
$$

$$
\theta_d = atan2(y_d, x_d) \\
$$

$$
\omega_d = \frac{\ddot{y}_d. \dot{x}_d - \ddot{x}_d. \dot{y}_d}{\dot{x}_d^2 + \dot{y}_d^2}
$$

Define the tracking error relative to the robot's frame:

$$
e_x = cos(\theta) (x_d - x) + sin(\theta) (y_d - y)
$$

$$
e_y = -sin(\theta) (x_d - x) + cos(\theta) (y_d - y)
$$

$$
e_{\theta} = \theta_d - \theta
$$

These equations represent the positional error projected into the robot’s coordinate frame, which makes the control independent of the global orientation. This is useful because it avoids having to control in a global coordinate system where orientation causes nonlinear effects.

Control law:

$$
v = v_d cos(e_\theta) + k_1 e_x
$$

$$
\omega = \omega_d + k_2 e_y + k_3 sin(e_{\theta})
$$
