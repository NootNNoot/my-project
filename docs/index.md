---
layout: default
title: Towards Balancing a Double Pendulum on a Custom 6-DoF Arm with LQR Control
---

# Towards Balancing a Double Pendulum on a Custom 6-DoF Arm with LQR Control

## Abstract

Balancing underactuated systems is a classic controls problem with applications in robotics, aerospace, and dynamic manipulation. This project explored the development of a balancing controller for a custom 6-degree-of-freedom robot arm with a passive double pendulum attached to the end effector. The primary objective was to stabilize the pendulum in an upright configuration while simultaneously maintaining the arm near a nominal pose using torque-based control in ROS 2 and Gazebo.

The project evolved through several major iterations, beginning with simple gravity compensation and proportional-derivative balancing, then progressing toward a model-based Linear Quadratic Regulator framework using rigid-body dynamics from Pinocchio. Significant effort was spent on dynamics modeling, controller architecture, gravity compensation through passive links, state representation, and stabilizing the interaction between the robot arm and the underactuated pendulum system.

Although a fully stable balancing controller was not achieved, the project successfully demonstrated:

- Accurate gravity compensation for the arm-pendulum system
- Online torque control through ROS 2 effort interfaces
- Simulation infrastructure for repeated balancing experiments
- Automated reset functionality in Gazebo
- Finite-difference linearization of the nonlinear dynamics
- Generation of a valid LQR gain matrix for the complete coupled system

The final system represents substantial progress toward a fully operational balancing controller and provides a strong foundation for future work in optimal control and nonlinear robotics.

---

## Introduction and Motivation

Balancing a pendulum is one of the most fundamental nonlinear controls problems in robotics. Systems such as inverted pendulums, reaction wheel pendulums, and cart-pole systems are widely used to evaluate control methods because they are inherently unstable and highly sensitive to disturbances.

My motivation for this project came from two videos: [Finding Order in Double Pendulum Chaos](https://www.youtube.com/watch?v=8jVogdTJESw) and [Showing the Beauty of Double Pendulums](https://www.youtube.com/watch?v=dtjb2OhEQcU). These videos introduced me to chaos theory, double pendulum modeling, and the beautiful patterns that can emerge from systems that initially appear unpredictable. Combined with my interest in robotic arms and manipulation tasks, I wanted to explore whether a custom robot arm could partially tame the chaotic behavior of a double pendulum.

This project investigated a significantly more difficult version of the standard pendulum balancing problem: balancing a double pendulum attached to the end effector of a custom 6-DoF robotic manipulator. Unlike a standard cart-pole system, the pendulum dynamics are coupled to a high-dimensional articulated arm whose motion directly affects the pendulum state. The problem also exists in three dimensions rather than being restricted to a simple planar or one-dimensional track.

The resulting system contains:

- Six actuated arm joints
- Passive pendulum dynamics
- Nonlinear rigid-body coupling
- Gravity effects
- Underactuation
- Unstable equilibrium points

The primary goal was to stabilize the pendulum in the upright configuration while allowing the arm to maintain a reasonable nominal posture.

For the project architecture, I used Ubuntu 24.04 and ROS 2 Jazzy as the operating system and robotics middleware, Gazebo Sim as the simulator, and Python for the main control code. I also used rigid-body dynamics libraries such as KDL and Pinocchio to compute useful transformations, gravity terms, and system dynamics.

---

## System Architecture

> **Note:** Insert images of the robot, Gazebo simulation, and controller pipeline here.

### Robot Structure

The system consists of:

- A custom 6-DoF serial manipulator
- An attached passive pendulum mechanism
- ROS 2 control interfaces
- A Gazebo physics simulation

The pendulum was implemented by replacing the original gripper with a passive linkage structure. In the final design:

- `joint6` acts as the first pendulum rotational axis.
- `pendulum_joint_2` acts as the second passive pendulum axis.

The desired equilibrium configuration was:

$$
\theta_{joint6} = 0
$$

$$
\theta_{pendulum\_joint\_2} = 0
$$

This configuration describes the pendulum pointing upward while the arm remains near its nominal pose.

---

### ROS 2 Control Architecture

The control pipeline used:

- `/joint_states` for state feedback
- Effort-based controllers for torque commands
- A custom balancing node implemented in Python

The controller published torque commands to:

```text
/arm_effort_controller/commands
```

This topic exposed the arm's effort interfaces and allowed the controller to send direct torque commands to the six actuated joints.

The full state vector was defined as:

$$
x =
\begin{bmatrix}
q_1 &
q_2 &
q_3 &
q_4 &
q_5 &
q_6 &
q_p &
\dot{q}_1 &
\dot{q}_2 &
\dot{q}_3 &
\dot{q}_4 &
\dot{q}_5 &
\dot{q}_6 &
\dot{q}_p
\end{bmatrix}^T
$$

where $q_1 \dots q_6$ are the arm joint positions, $q_p$ is the passive pendulum joint position, and the remaining terms are the corresponding velocities. This gives a 14-dimensional state vector for the system.

---

## Controller Breakdown

### Gravity Compensation Controller

Before attempting LQR, I first implemented gravity compensation so the robot arm could hold itself near a nominal pose without sagging or drifting too far. This reduced noise in the system and made later balancing experiments more meaningful.

To compute gravity compensation, I used the PyKDL library to extract the kinematic chain of the robot and compute rigid-body gravity torques using:

```python
PyKDL.ChainDynParam
```

This produced the gravity torque vector:

$$
\tau_g(q)
$$

The base controller then computed:

$$
\tau_{base} =
g_{scale}\tau_g(q)
-
D\dot{q}
+
\tau_{hold}
$$

where:

- $g_{scale}$ is a scalar multiplier used to tune gravity compensation.
- $D$ is a damping matrix.
- $\dot{q}$ is the arm joint velocity vector.
- $\tau_{hold}$ is a weak pose-holding torque that keeps the arm near a nominal configuration.

The pose-holding torque was computed as:

$$
\tau_{hold} =
-K_{hold}(q - q_{nom})
$$

where $K_{hold}$ is a diagonal gain vector and $q_{nom}$ is the desired nominal arm pose.

Initially, gravity compensation only considered the arm itself. However, this caused severe inaccuracies because the pendulum mass created additional torques on the wrist and upstream arm joints.

The gravity chain was later extended through the passive pendulum links:

```text
base_link → joint1 → joint2 → joint3 → joint4 → joint5 → joint6 → pendulum_joint_2 → final pendulum link
```

This allowed the gravity model to include the mass and inertia of the passive pendulum. Even though the pendulum joint was not directly actuated, its mass still affected the torques required by the arm.

The final implementation computed gravity torques through the full arm-pendulum chain but only published the first six torques corresponding to the actuated arm joints.

---

### Challenges with Passive Links

One major challenge was that the pendulum joints were passive and not directly controllable. However, their inertial contribution still affected the arm dynamics.

This created several issues:

- Overcompensation at the wrist
- Unstable oscillations
- Large torque spikes caused by improper gravity modeling
- Shape mismatches between full-chain gravity torques and the six-dimensional arm command vector

Several iterations were required to correctly:

- Include passive link inertias
- Separate controllable and uncontrollable joints
- Map gravity torques back to the six actuated arm joints
- Tune gravity scaling after the pendulum mass was included

This became one of the most important parts of the project. Without accurate gravity compensation, every balancing controller behaved unpredictably.

---

## PD-Based Balancing

### Initial Pendulum Balancing

After basic gravity compensation, I introduced proportional-derivative balancing logic to try to stabilize the goal joints. The first balancing controller used the following PD law:

$$
u =
-k_p\theta_p
-
k_d\dot{\theta}_p
-
k_{j6}\theta_{j6}
-
k_{dj6}\dot{\theta}_{j6}
$$

where:

- $\theta_p$ is the passive pendulum angle.
- $\theta_{j6}$ is the wrist angle.
- $\dot{\theta}_p$ is the passive pendulum angular velocity.
- $\dot{\theta}_{j6}$ is the wrist angular velocity.

The resulting balancing action was distributed across multiple arm joints with the largest influence on pendulum motion:

$$
\tau_{balance}
=
\begin{bmatrix}
0.15u &
0.35u &
0.35u &
0 &
0 &
0.05u
\end{bmatrix}^T
$$

The full torque command was then:

$$
\tau =
\tau_{base}
+
\tau_{balance}
$$

This controller was simple, but it helped reveal useful properties of the system.

---

### Emergent Swing-Up Behavior

One particularly interesting result was that increasing derivative gains on `joint6` unintentionally created a natural swing-up controller.

The arm began injecting energy into the pendulum through wrist oscillations and occasionally brought the pendulum near the upright configuration. This was not originally planned, but it showed that the arm-pendulum system could transfer energy effectively through the wrist and larger arm joints.

However, the controller could not reliably stabilize the pendulum after swing-up. The same aggressive gains that helped swing the pendulum upward caused oscillations near the equilibrium point. The arm also drifted away from its nominal pose during repeated balancing attempts.

This highlighted a fundamental limitation of the hand-tuned PD approach:

- Swing-up required aggressive energy injection.
- Stabilization required precise damping and smaller corrections.
- A single fixed-gain PD controller struggled to handle both regimes.

---

## Transition to LQR

The PD controller relied heavily on manual tuning and lacked a true model of the coupled arm-pendulum dynamics. This motivated a transition toward Linear Quadratic Regulation, which computes feedback gains from a linearized model of the system.

The nonlinear dynamics can be written as:

$$
\dot{x} = f(x,u)
$$

Near an equilibrium point, the system can be approximated by a discrete-time linear model:

$$
x_{k+1} = A x_k + B u_k
$$

where:

- $x_k$ is the state vector at timestep $k$.
- $u_k$ is the control input.
- $A$ describes how the state evolves without control.
- $B$ describes how the control input affects the state.

The LQR control law is:

$$
u_k = -Kx_k
$$

where $K$ is chosen to minimize the quadratic cost function:

$$
J =
\sum_{k=0}^{\infty}
\left(
x_k^T Q x_k
+
u_k^T R u_k
\right)
$$

Here:

- $Q$ penalizes state error.
- $R$ penalizes control effort.
- Larger values in $Q$ force the controller to care more about those state variables.
- Larger values in $R$ discourage large torque commands.

---

## Pinocchio-Based Dynamics Modeling

KDL was useful for gravity compensation, but it was not sufficient for the finite-difference dynamics linearization needed for LQR. For this reason, I transitioned to using Pinocchio for dynamics modeling.

Pinocchio was used for:

- Forward dynamics
- Rigid-body dynamics
- Articulated-body algorithms
- State-space linearization
- Handling joint configuration representations

The LQR generation script implemented:

- Automatic state extraction
- Finite-difference linearization
- Discrete-time system generation
- Riccati equation solving
- Gain matrix export to a `.npy` file

---

## Finite-Difference Linearization

The linearized matrices $A$ and $B$ were estimated numerically. For each state perturbation, the script computed:

$$
A_i =
\frac{
f(x + \epsilon e_i, u)
-
f(x - \epsilon e_i, u)
}{
2\epsilon
}
$$

For each input perturbation, the script computed:

$$
B_j =
\frac{
f(x, u + \epsilon e_j)
-
f(x, u - \epsilon e_j)
}{
2\epsilon
}
$$

This produced:

$$
A \in \mathbb{R}^{14 \times 14}
$$

and:

$$
B \in \mathbb{R}^{14 \times 6}
$$

The system was linearized around the nominal configuration:

$$
q_{nom}
=
\begin{bmatrix}
0 &
0.79 &
0.79 &
0 &
0 &
0 &
0
\end{bmatrix}^T
$$

This corresponds to the six arm joints plus the passive pendulum joint.

---

## LQR Gain Generation

To compute the LQR gain matrix $K$, the project solved the Discrete-Time Algebraic Riccati Equation using SciPy's `solve_discrete_are()` function.

The Riccati equation produces the matrix $P$, which is then used to compute:

$$
K =
\left(
B^T P B + R
\right)^{-1}
B^T P A
$$

The final gain matrix had the shape:

$$
K \in \mathbb{R}^{6 \times 14}
$$

This maps the 14-dimensional state vector to six arm joint torques.

Several challenges appeared during LQR gain generation:

- Continuous joint representation
- Unstable linearization around the nominal point
- Incorrect handling of revolute joints
- Passive pendulum dynamics
- Ensuring the model used the same state convention as the ROS 2 controller

One major issue was that Pinocchio represented some continuous revolute joints internally using:

$$
\begin{bmatrix}
\cos(\theta) &
\sin(\theta)
\end{bmatrix}
$$

instead of storing the angle directly. This caused an initial mismatch between the expected state dimension and the actual configuration representation. After correcting the state extraction logic, the script successfully generated a valid LQR gain matrix.

---

## Gazebo Reset Plugin

To support repeated testing, I implemented a custom Gazebo reset plugin. This plugin reset the robot joints to a known starting configuration without requiring a full simulator restart.

The plugin reset:

- The six arm joints
- The passive pendulum joint
- Joint velocities

The reset behavior was especially important because balancing experiments are sensitive to initial conditions. Without a repeatable reset procedure, it was difficult to compare controller changes fairly.

The plugin subscribed to a Gazebo transport topic and applied joint position and velocity reset components during the simulation update loop.

This made it possible to repeatedly test:

- Gravity compensation
- PD swing-up behavior
- LQR catch behavior
- Controller gain changes

---

## Logging and Evaluation

To evaluate controller behavior, I also created a logging script that subscribed to `/joint_states`, recorded joint angles over time, and plotted the resulting trajectories.

The logged state included:

- `joint1`
- `joint2`
- `joint3`
- `joint4`
- `joint5`
- `joint6`
- `pendulum_joint_2`

This allowed the system response to be visualized as a function of time. In particular, plotting `joint6` and `pendulum_joint_2` helped show whether the controller was reducing oscillations and keeping the pendulum near the desired upright position.

A useful evaluation plot would show the pendulum state converging toward:

$$
\theta_{pendulum\_joint\_2} = 0
$$

with decreasing oscillation amplitude over time.

---

## Results

### Successes

The project successfully achieved:

- Robust gravity compensation
- Stable torque-level arm control
- Coupled rigid-body dynamics modeling
- Finite-difference linearization
- Valid LQR gain generation
- Automated simulation resets
- Joint-angle logging and visualization

The arm was able to:

- Maintain nominal poses
- Inject swing-up energy into the pendulum
- Occasionally bring the pendulum near upright
- Generate a valid model-based LQR gain matrix

---

### Remaining Challenges

Although significant progress was made toward an LQR controller with gravity compensation, several challenges prevented the system from achieving true stabilization:

- Highly nonlinear dynamics
- Difficult underactuated coupling
- Sensitivity near equilibrium
- Instability during the transition from swing-up to stabilization
- Difficulty tuning the balance controller without exciting the full arm
- Passive pendulum dynamics that could not be directly actuated

The primary challenge was that swing-up and stabilization required very different controller behavior. Swing-up required aggressive energy injection, while stabilization required precise damping and small corrective torques.

The controller frequently overshot the equilibrium position, destabilized the wrist, or drifted away from the nominal arm configuration.

---

## Future Work

This project demonstrated how quickly robotic control problems become difficult when moving from low-dimensional systems to high-dimensional articulated manipulators.

Although a fully stable balancing controller was not achieved, the project made significant progress toward a complete solution.

Future work would include:

- Full-state LQR using improved equilibrium selection
- Gain scheduling between swing-up and stabilization modes
- Model Predictive Control instead of fixed-gain LQR
- Reinforcement learning approaches
- Trajectory optimization
- Improved state estimation in the world frame
- Better mode switching between energy injection and stabilization
- Real-time visualization of pendulum orientation and energy

A promising future direction would be to combine an energy-based swing-up controller with a locally stabilizing LQR controller around the upright equilibrium:

$$
\tau =
\begin{cases}
\tau_{swingup}, & \text{if the pendulum is far from upright} \\
\tau_{base} - Kx, & \text{if the pendulum is near upright}
\end{cases}
$$

This would allow the controller to use aggressive motion when the pendulum is far from the goal and precise linear feedback once the pendulum is close enough for LQR to be valid.

---

## Conclusion

This project explored the development of a model-based balancing controller for a custom 6-DoF robot arm with a passive double pendulum attached to its end effector. While the final system did not achieve fully stable balancing, the project made meaningful progress toward that goal.

The most important accomplishments were the development of a working gravity compensation controller, the inclusion of passive pendulum dynamics in the gravity model, the construction of a repeatable Gazebo testing workflow, and the generation of a valid LQR gain matrix using Pinocchio and finite-difference linearization.

The project showed that balancing a double pendulum with a full robotic arm is significantly more difficult than standard pendulum balancing problems because the controller must manage nonlinear dynamics, underactuation, gravity, passive link motion, and high-dimensional arm movement simultaneously.

Overall, the project provides a strong foundation for future work in optimal control, nonlinear robotics, and dynamic manipulation.
