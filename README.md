# Towards Balancing a Double Pendulum on a Custom 3D ARM with LQR Controls

## Abstract

Balancing underactuated systems is a classic controls problem with applications in robotics, aerospace, and dynamic manipulation. This project explored the development of a balancing controller for a custom 6-degree-of-freedom (6-DoF) robot arm with a passive double pendulum attached to the end effector. The primary objective was to stabilize the pendulum in an upright configuration while simultaneously maintaining the arm near a nominal pose using torque-based control in ROS2 and Gazebo.

The project evolved through several major iterations, beginning with simple gravity compensation and proportional-derivative (PD) balancing, then progressing toward a model-based Linear Quadratic Regulator (LQR) framework using rigid-body dynamics from Pinocchio. Significant effort was spent on dynamics modeling, controller architecture, gravity compensation through passive links, state representation, and stabilizing the interaction between the robot arm and the underactuated pendulum system.

Although a fully stable balancing controller was not achieved, the project successfully demonstrated:

- Accurate gravity compensation for the arm-pendulum system
- Online torque control through ROS2 effort interfaces
- Simulation infrastructure for repeated balancing experiments
- Automated reset functionality in Gazebo
- Finite-difference linearization of the nonlinear dynamics
- Generation of a valid LQR gain matrix for the complete coupled system

The final system represents substantial progress toward a fully operational balancing controller and provides a strong foundation for future work in optimal control and nonlinear robotics.

## Introduction and Motivation 

Balancing a pendulum is one of the most fundamental nonlinear controls problems in robotics. Systems such as inverted pendulums, reaction wheel pendulums, and cart-pole systems are widely used to evaluate control methodologies because they are inherently unstable and highly sensitive to disturbances.

My main motivation for this project came out from these two videos: [Finding order in double pendulum choas](https://www.youtube.com/watch?v=8jVogdTJESw) and [showing the beauty of double pendulums](https://www.youtube.com/watch?v=dtjb2OhEQcU). These two videos were my big introduction to chaos theory, as well as the modeling of double pendulums and showing how beautiful patterns emerge from them despite how chaotic they may first seem. Combined with my passion for robotic arms and manipulation tasks, I wanted to see if I could also tame the chaos that these double pendulums present by stabalizing one with a custom robot arm. 

From the previous problem statement, this project investigated a significantly more difficult variation of the problem: balancing a double pendulum attached to the end effector of a custom 6-DoF robotic manipulator. Unlike a standard cart-pole system, the pendulum dynamics are coupled to a high-dimensional articulated arm whose motion directly affects the pendulum state, as well as the problem being now in 3 dimensions instead of 2 (or really 1 since a cart is usually just on a single line)

My resulting system then contains:

- Six actuated arm joints
- Passive pendulum dynamics
- Nonlinear rigid-body coupling
- Gravity effects
- Underactuation
- Unstable equilibrium points

The primary goal was to stabilize the pendulum in the upright configuration while allowing the arm to maintain a reasonable nominal posture.

For the project architecture, I used Ubuntu 24.04 and ROS2 Jazzy as the OS and system design, with Gazebo Sim as my simulator and all code was written in Python. I also used rigid-body dynamics libraries like KDL and Pinocchio to calculate useful transformations and properties. 

## System Architecture
(NOTE: INSERT IMAGES OF ROBOT)

### Robot Structure

The system consists of:

- Acustom 6-DoF serial manipulator,
- An attached passive pendulum mechanism,
- ROS2 control interfaces,
- Gazebo physics simulation.

The pendulum was implemented by replacing the original gripper with a passive linkage structure. In the final design:

joint6 acts as the first pendulum rotational axis,
pendulum_joint_2 acts as the second passive pendulum axis.

The desired equilibrium configuration was: 

$\theta_{joint6} = 0$ 

$\theta_{pendjoint2} = 0$

Which describes the pendulum pointing upwards in the world frame

### ROS2 Control Architecture

The control pipeline used:

- /joint_states for state feedback
- Effort-based controllers for torque commands
- Custom balancing node implemented in Python

The controller published torques to:

/arm_effort_controller/commands

which exposed the joint interfaces to be able to accept tourque commands for precise control. 

We defined the full state vector to be 

```math
x = \begin{bmatrix} q_1 & q_2 & q_3 & q_4 & q_5 & q_6 & q_p & \dot{q_1} & \dot{q_2} & \dot{q_3} & \dot{q_4} & \dot{q_5} & \dot{q_6} & \dot{q_p} \end{bmatrix}^T
```

where $$q_1 \cdots q_6$$ are joint positions and $q_p$ is the pendulum position, and the rest are the velocities, which gives us a 14 state vector to describe the system

## Controller Breakdown

### Gravity Compensation Controller

Before being able to start LQR, I had to implement gravity compensation within the arm so that it is able to hold itself up near a nominal pose and not drift too far to reduce the most amount of noise possible in the system. TO do this, I used the PyKDL library to get the kinemtatic chain of the robot via 
` 
PyKDL.ChainDynParam
`

This gave the rigid body gravity tourques via $$\tau_g(q)$$ which the controller then used to compute 

```math
\tau = g_{scale}\tau_g - D\dot{q}
```

where 
- $$g_{scale}$$ is a constant scalar for the gravity to further stabalize the arm
- $$D$$ is a damping matrix
- $$\dot{q}$$ is the joint velocity vector

Initially, gravity compensation only considered the arm itself. However, this caused severe inaccuracies because the pendulum mass created additional torques on the wrist joints.

The gravity chain was later extended through the passive pendulum links from the base link to properly counteract all masses and inertias acting on the arm

#### Challenges with the Passive Links

One major challenge was that the pendulum joints were passive and not directly controllable. However, their inertial contribution still affected the arm dynamics.

This created several issues:

- Overcompensation at the wrist
- Unstable oscillations
- Large torque spikes caused by improper gravity modeling.

Several iterations were required to correctly:

- Include passive link inertias
- Separate controllable and uncontrollable joints,
- Correctly map gravity torques to the arm controller.

## PD-Based Balancing

### Initial Pendulum Balancing

After basic gravity compensation, I introduced basic PD balancing logic to try and stabilize the two goal joints. The first attempt was to us this PD Law: 

$$u = -k_p\theta_p - k_d\dot{\theta_p} - k_{j6}\theta_{j6} - k_{dj6}\dot{\theta_{j6}}$$

where 
- $$\theta_p$$ is the pendulum angle
- $$\theta_{j6}$$ is the wrist angle
- Rest of the terms are angular velocities

The resulting balancing was distributed across multiple arm joints with the highest impacts on pendulum movements: 

$$\tau = \begin{bmatrix} 0.15u & 0.35u & 0.35u & 0 & 0 & 0.05u \end{bmatrix}$$

#### Emergent Swingup Behaviour

One particularly interesting result was that increasing derivative gains on joint6 unintentionally created a natural swing-up controller.

The arm began injecting energy into the pendulum through wrist oscillations, occasionally bringing the pendulum near the upright configuration.

However, the controller could not stabilize the pendulum after swing-up because the gains became too aggressive near equilibrium, oscillations grew uncontrollably, and the arm drifted away from its nominal pose. This highlighted a fundamental limitation of the hand-tuned PD approach.

## Transition to LQR 

The PD controller relied heavily on manual tuning and lacked a true model of the coupled arm-pendulum dynamics.

This motivated a transition toward Linear Quadratic Regulation (LQR), which computes optimal feedback gains for a linearized system.

Since we can model the continuous nonlinear dynamics via $$\dot{x} = f(x, u)$$< we can approximate near an equilibrium with

$$x_{k+1} = Ax_k + Bu_k$$

where the optimal control law then becomes $$u = -Kx$$ where $$K$$ minimizes the quadratic cost

$$J = \sum{x^T_kQx_k + u^T_kRu_k}$$

### Pinocchio Based Dynamics Modeling

Since KDL was insufficient for the required dynamics linearization, I turned to using Pinocchio for forward dynamics, rigid-body dynamics, articulated-body algorithms, and state-space linearization tools.

In which I implemented:

- Finite-difference linearization
- Automatic state extraction
- Discrete-time system generation

The finite difference linearization worked to estimate the $$A$$ and $$B$$ matrices in the LQR equation by perturbing the initial state and simulating the changes, where

$$A_i = \frac{f(x + \epsilon e_i, u) - f(x - \epsilon e_i, u)}{2\epsilon}$$

$$B_j = \frac{f(x, u  + \epsilon e_j) - f(x, u - \epsilon e_j)}{2\epsilon}$$

which produced a 14x14 state transition matrix $$A$$ and a 14x6 control matrix $$B$$. 

The system was then linearized around the nominal configuration: 

$$q_{nom} = \begin{bmatrix} 0 & 0.79 & 0.79 & 0 & 0 & 0 & 0 \end{bmatrix}$$

### LQR Gain Generation 

To get the gain matrix $$K$$, we solve the Discrete-Time Algebraic Riccati Equation (DARE) with
`solve_discrete_are()` from SciPy. 

This gain matrix is of the form: 

$$K = (B^TPB + R)^{-1}B^TPA$$

From this, several challenges did still happen via
- Continuous joint representation
- Unstable Linearization
- Incorrect Handling of Revolute Joints

## Results

### Succusses

The project successfully achieved:

- Robust gravity compensation
- Stable torque-level arm control
- Coupled rigid-body dynamics modeling
- Finite-difference linearization
- Valid LQR gain generation
- Automated simulation resets

The arm was able to:

- Maintain nominal poses
- Inject swing-up energy
- Occasionally bring the pendulum near upright

### Remaining Challenges

Even though a lot of preogress was made in building the LQR controller with gravity compensation, a lot of the challenges I faced during the project blocked me from acheiving true stabilization such as: 

- Highly nonlinear dynamics,
- Difficult underactuated coupling
- Sensitivity near equilibrium,
- Instability during the transition from swing-up to stabilization.

The primary challenge was that:

- Swing-up required aggressive energy injection,
- While stabilization required extremely precise damping.

The controller frequently overshot the equilibrium position, destabilized the wrist, and drifted away from the nominal arm configuration due to challenges stabilizing the pendulum

## Future Work

This project demonstrated how rapidly robotic control problems become difficult when moving from low-dimensional systems to high-dimensional articulated manipulators.

Although a fully stable balancing controller was not achieved, the project made significant progress toward a complete solution.

Future work would include:

- Full-state LQR using improved linearization,
- Using Model Predictive Control (MPC) instead of LQR
- Reinforcement learning approaches
- Trajectory optimization
- Improved state estimation in the world frame
