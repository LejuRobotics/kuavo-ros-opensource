# Task 2 motion plan

> 上升 IK 的当前候选设计、验证边界和修改纪律见
> `TASK2_LIFT_IK_PLAN.md`。该方案尚未通过正式运行验收，不是最终可行方案。

## Scope and coordinate rules

Task 2 moves both `box_1` and `box_2` in sequence from `work_table` to
`destination_conveyor`.  The conveyor advances only the first released box to
its far end so the same placement region is clear for the second box, then
stops before the second release.  World `+X` is the robot's initial forward direction,
world `+Y` is its initial left direction, and `+Z` is up.

All object and base poses used for decisions come from live MuJoCo pose topics.
Nominal MJCF coordinates are scene defaults, not task-state truth.  Every arm
IK solve uses:

1. the measured base pose after chassis motion has stopped;
2. the selected box's measured pose;
3. the measured initialized arm joints as the first solver seed; and
4. the dedicated `box_gripper_pose`, never the older double-gripper TCP.

The task must solve every waypoint in a stage before commanding the first
waypoint of that stage.  An IK or collision failure therefore cannot leave the
arms halfway through an unverified motion.

## Reproducible initial state

`TASK_SEED` drives one local `random.Random` instance.  The base starts with
identity yaw and unchanged height.  Its translation ranges are:

- lateral: world `Y in [-0.10, 0.10] m`;
- longitudinal: world `X in [-0.03, 0.03] m`.

The lateral range is the user-selected 10 cm each way.  The smaller
longitudinal range is initially 3 cm each way and remains an explicit policy
constant so validation can change it without hiding the choice.

Scene 2 has a free mobile base and no `scene_base_lock`.  Its base translation
must be injected into `/robot_init_state_param` through launch arguments before
the controller and MuJoCo initialize.  `/set_object_position` is not used for
this: a post-launch free-joint write is immediately opposed by the controller's
already-created origin reference.

## State machine

### 1. Observe and initialize

1. Set the reproducible random base translation before any arm command.
2. Read and save the randomized world base pose `B0`, both boxes, and all 14
   arm joints.
3. Compute `Bsafe` by moving the configured fixed distance backward in the
   initial base frame.  The fixed distance, arm-lift path, return path and declared
   random envelope must already have passed offline clearance validation with
   margins before this configuration can be approved for runtime.
4. Move once to `Bsafe` and re-read the measured arm state.
5. Leave both hands in their measured initial posture and copy all 14 measured
   arm joints.  Replace only `zarm_l1_joint` and `zarm_r1_joint` with the common
   configured shoulder-pitch lift target, then execute that single trajectory.
   The other 12 arm joints and all hand joints remain unchanged; there is no
   complete-arm ready target and no box-gripper shaping during initialization.
6. Return once to the saved absolute world pose `B0`; this is not an open-loop
   forward move equal to the retreat distance.  Do not add task-level arrival
   rechecks, corrective moves, or retry branches.
7. Read the actual returned base pose `B1`, boxes and achieved arm joints.
   Store those measured values for the comfortable-workspace decision and use
   the 14 measured joints as the first grasp IK seed.  Small `B1-B0` control
   error is part of the randomized state and is covered by workspace margins.

The fixed retreat distance and shoulder-pitch lift target live in
`config/task2_initialization.json`.  They are explicitly marked as requiring
offline and later runtime validation before approval.  The runtime action
itself is deterministic and contains no collision-search or return-correction
fallback.

### 2. Select a front docking pose and lock the box

For each box independently:

1. Generate a front docking target in the measured box frame.  With the current
   boxes facing world `+X`, the target aligns base `Y` with the selected box and
   sets base `X` from the required front standoff; it does not aim through the
   narrow gap between the boxes.
2. Compare the chassis displacement from the actual returned pose `B1` to the
   two front targets.  If `B1` is already inside one target's robust front
   region, keep the chassis still and select that box.  Otherwise select the
   reachable front target requiring less chassis motion.
3. Move to the selected front target, stop, and re-read the measured base and
   selected-box poses.  Lock that box as the only grasp target for the rest of
   the pick stage.

Box choice therefore belongs to chassis docking, not arm IK.  Once the base is
in front of one box, the IK planner solves only that box.  It must not compare
the other box and silently switch targets after docking.

### 3. Approach from above and grasp

After reaching the selected box's front docking pose, stop the chassis and
re-read the measured base and box poses.  Solve only the locked, directly
facing box rather than replaying an offline candidate or choosing again.

1. Starting from the measured initialized arm joints and unchanged hand pose,
   solve a clearance waypoint in front of the box, then the waypoint above the
   two selected box contact regions.  At both waypoints, both r7 local `-Z`
   axes (the finger reach direction) point along the box forward axis; the two
   hands are never oriented toward one another.
2. At that solved arm waypoint, shape both hands with
   `box_gripper_pose(closure=0)` without changing the arm target.
3. Solve a vertical descent to the contact waypoint while preserving the
   selected wrist orientations.
4. Command the descent path.
5. Close both hands along the one-dimensional box path to `closure=1`.  The
   bimanual contact latch fixes the box and disables the named fingertip
   collision masks, while the hand controller completes and then continuously
   publishes the full closed posture so all four fingers finish the grasp.
6. Confirm the grasp from the explicit bimanual contact-latch topic, then use
   the candidate terminal-pose IK documented in `TASK2_LIFT_IK_PLAN.md`.
   This solves one explicit pair of final `hand_base` SE(3) targets; its nine
   published points are joint-space interpolation, not nine constrained IK
   solves.  Record the measured box rise without adding a separate height
   threshold.  Contact is only the latch trigger: once the box is
   latched, disable the named fingertip collision masks so MuJoCo's contact
   solver cannot fight the kinematic box follower and shake the grippers.
   Restore the original masks on explicit release so the next box can latch.

Task 1 contributes the pattern “measured joint seed -> solve above -> descend ->
close -> verify -> lift”.  Task 2 does not reuse its single-arm cylinder TCP,
fixed joint-space lift, or right-hand two-pad latch rule.

The current staged implementation is `examples/task2_pick.py`, configured by
`config/task2_pick.json`.  It requires the B1 parameters written by
`task2_initialize.py`.  Initialization raises both arms and, without lowering
the hand origins, turns both palms toward the table into the same orientation
used by the grasp planner.  The pick stage therefore starts directly at the
`above` target instead of executing a separate orientation-clearance waypoint.
It chooses the front docking target without arm IK, locks
the selected box after docking, then uses `utils/task2_pick_planner.py` to solve
and collision-check the complete above/descent/closure/lift sequence before
the first arm command.

The formal full-flow entry is `examples/task2.py`: it runs initialization once,
then runs the same pick/transport/direct-release stage twice in the same Scene
2 ROS graph.  It writes `task_result.txt=success` only after both subprocesses
finish and `/task2_placed_box_count` equals two; any stage failure writes
`task_result.txt=fail` and stops the sequence without retry or fallback.

### 4. Lift and transport

1. Execute the candidate terminal-pose lift defined in
   `TASK2_LIFT_IK_PLAN.md`; do not treat it as accepted until its staged
   runtime checks pass.
2. Back the chassis away from `work_table` until the held box clears the table
   footprint and the unselected box.
3. Rotate left toward `destination_conveyor` only after this clearance condition
   is met.
4. Use the configured absolute destination docking pose for the conveyor
   scene pose/configuration.  Move to that pose; do not accumulate fixed
   relative displacements from the randomized start.  The current pose is
   `(-0.20, 0.30, 90 deg)`, placing the chassis `0.10 m` closer to the belt
   than the former table-center-matched pose.
### 5. Place and score

1. Keep the arms at the lifted `above` waypoint after the chassis stops; do not
   add a placement descent or a post-placement lift.
2. For the first box, explicitly release the box latch and open both grippers
   to `closure=0` so the box falls onto the stationary belt.  After the release
   settles, return both arms to the stored initialization `ready` posture, then
   enable the conveyor.  With the arms retracted, rotate toward B1 directly;
   do not add the former post-release chassis retreat.  Move the box to
   conveyor-local `x=0.65 m` to leave extra clearance for the second box.
3. Return to B1 and repeat the same pick/transport sequence for the remaining
   box.  Before releasing the second box, disable the conveyor, then release
   the latch and open the grippers.  The second box remains at the near end.
   After the free fall settles, return both arms to the same stored `ready`
   posture before any chassis motion.  Then execute the configured `0.20 m`
   retreat, rotate, and return to B1.

## Scene-change gate

Changing the scene is allowed only after the new box posture, wrist/contact
targets, docking search, and collision checks have been exercised across the
declared random base bounds.  A scene change must cite which constraint has no
feasible solution.  Allowed first adjustments are box lateral spacing or box
placement on `work_table`; robot model, hand model, task objects, and toolchain
remain unchanged.

## Staged acceptance

1. Random base set/readback, fixed safe retreat, single complete-arm ready, and
   settled return to the saved randomized pose, with no box motion.
2. Offline two-box docking and full above/descent/lift IK over boundary poses.
3. One fixed-seed grasp and lift probe, with no chassis transport.
4. One fixed-seed loaded retreat, turn, and destination approach.
5. One fixed-seed placement and measured success result.
6. At least five seeds covering lateral and longitudinal extrema before the
   workflow is accepted for collection.
