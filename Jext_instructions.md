# CLIK1 with Extended Jacobian (J_ext) – Implementation Instructions

## Context

This project implements a **Closed-Loop Inverse Kinematics (CLIK)** controller for an **aerial manipulator** composed of:
- a free-flying base (UAV / drone),
- a serial robotic arm mounted underneath.

The control is formulated using:
- the **Centroidal Momentum Matrix (CMM)**,
- an **extended Jacobian (J_ext)**,
- and a **momentum-based task applied only to the manipulator subsystem**.

The base is not directly actuated by the CLIK controller and is eliminated through dynamic consistency.

---

## Constraint to stack to the Generalized Jacobian formula 

The hard constraint equation is:

$$
\left( A_{KO,b}^{man} A_{b}^{-1} A_{m} + A_{KO,m}^{man} \right)\,\dot{q}_m
=
K_{O}^{man}(t_{k})
+
(v_O \times \mathbf{p}_{man} + \tau_R + \tau_g)\,\Delta t
$$

where:
- $ q_m $ are the manipulator joint coordinates,
- $ A_{KO,b}^{man}, A_{KO,m}^{man} $ are submatrices of the **Momentum Matrix** that maps manipulator joint velocities to its angular momentum about the connection point to the drone $O$.
- $A_{b}, A_{m}$ are the are submatrices of the **Centroidal Momentum Matrix** of the entire aerial manipulator system. Those already used for the control that is now implemented in the python script you have to modify.
- $ K_{O} $ is the **angular momentum of the manipulator**, computed about the **drone-manipulator connection point** $O$. This point coincides with the origin of the frame `mobile_wx250s/base_link`. This can be computed from the *centroidal angular momentum* using the transport formula $K_{O}^{man} = K_{G_m}^{man} + G_mO \times \mathbf{p}_{man}$.
- $G_m$ is the centroidal of the manipulator.
- $\mathbf{p}_{man}$ is the linear momentum of the manipulator.
- $ \tau_R $ is the reaction torque vector between the manipulator and the drone base which we want to set to zero. So this term will be null.
- $ \tau_g $ is the gravity torque on the manipulator.

The controller acts **only on the arm**, but remains dynamically consistent with the floating base.

The constraint equation comes from the Euler dynamics equation and from the definition of angular momentum as K=A*q. The equation is needed in order to express the reaction torque between base and manipulator (which we want to minimize) as function of the joint velocities.

The $A_{KO}$ momentum matrix should be computable with the formula (please verify the correctness of this formula before implementing it)

$$A_{KO}^{man} = A_{K}^{man} + skew(G_mO)A_{p}^{man}$$

where:
- $A_K$ is the angular momentum block of the **Centroidal Momentum Matrix** of the single manipulator.
- $A_{p}$ is the linear momentum block of the **Centroidal Momentum Matrix** of the single manipulator.
- $skew$ is the skew-symmetric operator applied to the vector $G_mO$ connecting the manipulator centroid to the connection point $O$


---

## High-Level Control Strategy

1. Use **manipulator angular momentum equation** as a secondary task for the manipulator.
2. Construct an **extended Jacobian \( J_{ext} \)** that stacks:
   - end-effector kinematic constraints,
   - angular momentum constraint.
3. Solve a velocity-level (or acceleration-level) inverse kinematics problem as a **QP** problem.

---

## Required Measurements and Signals

Before computing the control law, the following quantities must be available and stored:

- Measured base velocities (linear + angular)
- Measured manipulator joint velocities \( \dot{q}_m \)

Joint velocities are obtained via **numerical differentiation** of measured positions.

---

## Momentum Matrices

After calling a function equivalent to compute the **Centroidal Momentum Matrices** (CMM) of the whole
system ($A$) and of the only manipulator system ($A^{man}$).

Then extract the following submatrices:

- $ A_{b} $: base block of the UAM CMM ([:],[1:6])
- $ A_{m} $: manipulator block of the UAM CMM ([:],[1:6])
- $ A_{KO,b}^{man} $: angular momentum–base block of the manipulator MM ([4:6],[1:6])
- $ A_{KO,m}^{man} $: angular momentum–base–manipulator block of the manipulator MM ([4:6],[7:(n_v-6)])

Compute the **manipulator angular momentum** using a function equivalent to `computeCentroidalMomentum` and extract $ K_{G_m} $. Then compute $K_{O}$

---

## Extended Jacobian Definition

The extended Jacobian is defined as:

$$
J_{ext} =
\begin{bmatrix}
J_{gen,lin} \\
A_{KO,b}^{man} A_{b}^{-1} A_{m} + A_{KO,m}^{man}
\end{bmatrix}
$$

where:
- $ J_{gen,lin} $ is the **linear part of the generalized Jacobian** mapping manipulator joint velocities to end-effector linear velocity.

---

## Desired Task Vector (redundant case)

The desired task velocity is:

$$
\dot{x}_{ee,des}
=
\begin{bmatrix}
\dot{x}_{ee,ref}+Ke \\
K_{O}^{man}(t_{k})+(v_O \times \mathbf{p}_{man} + \tau_R + \tau_g)\,\Delta t - A_{KO,b}^{man}A_{b}^{-1}\mathbf{h}_{\text{UAM}}
\end{bmatrix}
$$

where:
-  $e$  is the end-effector position error,
-  $K$  is a proportional gain,
- $\mathbf{h}_{\text{UAM}}$ is the total momentum of the aerial manipulator system

The final task vector is:


$$
\begin{bmatrix}
\dot{x}_{ee,ref} + K\,e_{lin} \\
K_{O}^{man}(t_{k})+(v_O \times \mathbf{p}_{man} + \tau_R + \tau_g)\,\Delta t - A_{KO,b}^{man}A_{b}^{-1}\mathbf{h}_{\text{UAM}}
\end{bmatrix}
$$


## Control Problem Formulation (redundant case)

The control problem is formulated as:

$\min \;\; \| J_{ext}\,\dot{q}_m - \dot{x}_{ee,des} \|$

embedded into a **QP** with joint limits and velocity bounds.

## Control Problem Formulation (EE position+orientation tracking case)

In this case I would like to track also a reference end-effector orientation trajectory (like it is done in `test2_Jgen_pinocchio`). So I would like to split two cost function. I would like to solve the QP problem:

$\min\limits_{\dot{q}_m} \ ( \| J_{gen}\,\dot{q}_m - \dot{x}_{ee,des} \|_{W_1} + \|(A_{KO,b}^{man} A_{b}^{-1} A_{m} + A_{KO,m}^{man})\dot{q}_m  - (K_{O}^{man}(t_{k})+(v_O \times \mathbf{p}_{man} + \tau_R + \tau_g)\,\Delta t) \|_{W_2})$

subject to the usual velocity and joint constraints. In this case we consider all the rows of the $J_{gen}$ matrix and of the $\dot{x}_{ee,des}$ vector. $\dot{x}_{ee,des}$ is defined as:

$$
\dot{x}_{ee,des}
=
\dot{x}_{ee,ref}+Ke
$$.

Add a parameter in order for the user to decide wether to use this or the $J_{ext}$ case.
Define also weights for the two cost functions so that the user can decide wether to give priority to the kinematic tracking or the minimization of the reaction torque. 

## Control Problem Formulation (minimization of reaction torque with kinematics as constraint)

I would like also to formulate the control problem in another way in the `test_reaction_torque.py` script. I would like to minimize the momentum cost function that has been used in the previous script using the kinematics equation as equality constraint. So the problem is formulated as:

$$\min\limits_{\dot{q}_m} \|(A_{KO,b}^{man} A_{b}^{-1} A_{m} + A_{KO,m}^{man})\dot{q}_m  - (K_{O}^{man}(t_{k})+(v_O \times \mathbf{p}_{man} + \tau_R + \tau_g)\,\Delta t) \|$$

$$\text{s.t.} \qquad [J_{gen}]\dot{q}_m=\dot{x}_{ee,des}$$

and the other inequality constraints related to arm joint limits. 

Define also here two cases:
- one in which the kinematic constraint is on both posiiton and orientation of the end-effector (full 6D tracking of EE pose).
- one in which the kinematic constraint is only on EE position.


---