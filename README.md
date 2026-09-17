# Modelling and Control of Manipulators — Exam Solution

**University of Genoa — DIBRIS**
Course: *Modelling and Control of Manipulators*
Exam session: 16/01/2026
Professors: Enrico Simetti, Giorgio Cannata
Tutors: Luca Tarasi, Simone Borelli

## Overview

This repository contains my MATLAB solution to the final exam exercise of the *Modelling and Control of Manipulators* course. It implements, from scratch (no built-in MATLAB kinematics functions), a full inverse-kinematics control loop for a 7-joint industrial manipulator — 6 revolute joints plus one prismatic joint (joint 6) — and uses it to drive first the **tool frame** `<t>` and then the **end-effector frame** `<e>` to two different goal poses.

The manipulator's geometry (link frames and dimensions) matches the CAD model given in the exam text (`Figure 1`).

## Problem statement

1. **Q1 — Cartesian error**: compute the pose error between the current tool frame `bTt` and the goal frame `bTg,tool`.
   - Goal orientation (YPR): `Γ_g,tool = [1.0363, 0.3218, -0.0077]` rad
   - Goal translation: `bO_g,tool = [0.0087, 0.5808, 0.6714]` m
2. **Q2 — Reference velocities**: from the pose error, compute the desired linear and angular reference velocity of the controlled frame w.r.t. the base.
3. **Q3 — Joint velocities**: invert the manipulator Jacobian to get the desired joint velocities `q̇` from the reference task-space velocity.
4. **Q4 — Simulation**: integrate `q̇` in time (Euler integration) to simulate the motion, moving the tool frame to the goal.
5. **Q5 — Second phase**: repeat Q1–Q4 controlling the **end-effector** frame instead, toward a second goal:
   - Goal orientation (YPR): `Γ_g,ee = [-0.3876, -0.3614, -0.3647]` rad
   - Goal translation: `bO_g,ee = [0.0202, 0.7779, 0.9927]` m

The control point is switched from tool to end-effector between the two phases by reusing the same control loop (no code duplication), by changing which frame's Jacobian/pose is used.

**Initial joint configuration:**
```
q0 = [π/2, -π/4, 0, -π/4, 0, 0.15, π/4]
```

## Repository structure

```
.
├── main.m                     # Entry point: runs both control phases
└── include/
    ├── BuildTree.m             # Static (q=0) homogeneous transforms between consecutive joint frames
    ├── geometricModel.m        # Direct geometry: forward kinematics + tool/end-effector pose
    ├── kinematicModel.m        # Geometric Jacobian of the tool/end-effector w.r.t. the base
    ├── cartesianControl.m      # Cartesian error + reference task-space velocity computation
    ├── KinematicSimulation.m   # Euler integration of joint velocities, with joint-limit clamping
    ├── RotToYPR.m               # Rotation matrix -> Yaw-Pitch-Roll angles
    ├── YPRToRot.m               # Yaw-Pitch-Roll angles -> rotation matrix
    └── utils/
        └── plotManipulators.m   # Provided plotting utilities (unmodified) — motion & velocity plots
```

## Module details

### `BuildTree.m`
Defines `iTj`, the tree of 7 static homogeneous transformations (one per joint) between consecutive frames, evaluated at `q = 0`, following the frame assignment in Figure 1 (base → joint 1 → … → joint 7). These are the fixed offsets used by `geometricModel` before the joint-dependent part of each transform is applied.

### `geometricModel.m`
A `handle` class holding the robot's kinematic structure and current state:
- `iTj_0`: the static transforms from `BuildTree`.
- `jointType`: per-joint type, `0` = revolute, `1` = prismatic (joint 6).
- `eTt`: fixed transform from the end-effector (flange) to the tool frame.
- `updateDirectGeometry(q)`: for each joint, builds the variable part of the transform (rotation about z for revolute joints, translation along z for the prismatic joint) and composes it with the static part to update `iTj`.
- `getTransformWrtBase(k)`: chains `iTj` from joint 1 to `k` to get `bTk`.
- `getToolTransformWrtBase()`: returns `bTt = bTe * eTt`, i.e. the tool pose obtained by extending the end-effector pose with the fixed tool offset.

### `kinematicModel.m`
A `handle` class wrapping a `geometricModel` instance and computing Jacobians:
- `getJacobianOfLinkWrtBase(i)`: builds the 6×n geometric Jacobian of link `i` w.r.t. the base, using the standard revolute/prismatic column rules (`v = z × (p_e - p_j)`, `w = z` for revolute; `v = z`, `w = 0` for prismatic).
- `updateJacobian()`: same construction but targeting the **tool tip** position (`p_tool` from `getToolTransformWrtBase`) instead of a link origin, so `self.J` is always the Jacobian of the currently controlled point (tool in phase 1, end-effector in phase 2, depending on how `eTt` is set at the time).

### `cartesianControl.m`
A `handle` class implementing the control law:
- Constructor takes the `geometricModel`, an angular gain `k_a`, and a linear gain `k_l`.
- `getCartesianReference(bTg)`:
  1. Computes the linear error `e_p = p_goal - p_tool` directly in the base frame.
  2. Computes the orientation error via axis-angle extraction: `R_err = R_tool' * R_goal`, angle `θ` from `trace(R_err)`, axis `n` from the skew-symmetric part of `R_err`, giving `e_o_tool = θ·n` in the tool frame, then rotated back into the base frame (`e_o = R_tool * e_o_tool`) since the Jacobian is expressed in the base frame.
  3. Scales both errors by the gains (`v_ref = k_l·e_p`, `w_ref = k_a·e_o`) and returns the stacked 6×1 reference velocity `x_dot = [v_ref; w_ref]`.

### `KinematicSimulation.m`
Simple Euler integrator: `q_new = q + q_dot * ts`, followed by clamping each joint to `[q_min, q_max]` to respect joint limits.

### `RotToYPR.m` / `YPRToRot.m`
Convert between a 3×3 rotation matrix and Yaw-Pitch-Roll angles (`ψ, θ, φ`), used to turn the goal orientations given as YPR triplets in the exam text into rotation matrices (`YPRToRot`) and, if needed, to inspect resulting orientations (`RotToYPR`). `RotToYPR` includes the singular-configuration check (`cos(θ) ≈ 0`) with a fallback for yaw/roll.

### `include/utils/plotManipulators.m`
Provided and unmodified. Plots the manipulator's link chain in 3D at each simulation step (`initMotionPlot`, `plotIter`) and, at the end of each phase, the final configuration together with the direction of the angular/linear velocity components over time (`plotFinalConfig`).

## How the two phases are handled without duplicating the loop

Both phases call the same control/simulation loop:
1. `km.updateJacobian()` — Jacobian of the currently controlled frame (tool or end-effector).
2. `cc.getCartesianReference(bTg)` — Cartesian error and reference velocity toward the current goal (`bTg,tool` or `bTg,ee`).
3. Joint velocities from the (pseudo-)inverse of the Jacobian applied to the reference velocity.
4. `KinematicSimulation` integrates the joint velocities and enforces joint limits.
5. `plotManipulators` updates the plots.

Between phase 1 and phase 2, only the controlled-frame offset (`eTt` in `geometricModel`, switched from the tool offset to identity/end-effector) and the goal pose (`bTg,tool` → `bTg,ee`) change — the loop body itself is not duplicated.

## Requirements

- MATLAB. No built-in kinematics/control toolbox functions are used; all direct geometry, Jacobian, and control computations are implemented manually, as required by the exam rules.

## Running the code

```matlab
main
```

This runs both phases in sequence:
1. Tool frame `<t>` converges to `bTg,tool`.
2. End-effector frame `<e>` converges to `bTg,ee`.

## Results

For each phase, `plotManipulators.plotFinalConfig` shows the manipulator's final configuration in 3D together with the normalized angular and linear velocity components over time, confirming that the Cartesian error is driven toward zero and the controlled frame converges to its goal pose.
