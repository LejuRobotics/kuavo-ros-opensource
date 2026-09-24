# Data Challenge Simulator — S400062 task scenes

This package follows the upstream `examples` collection workflow.  Task 1 now
uses one canonical scene and action entry.  The former legacy Task 1 entry has
been removed rather than kept as a second runnable route:

- robot: `biped_s400062_icra.xml` with Heiman SG100 hands;
- scene: `models/biped_s400062/xml/task1.xml`;
- action policy: `examples/task1.py` (one randomized object in each lateral
  half of the source bin);
- collection orchestrator: `examples/helperfunc.py`.

The supported workflow is:

```text
examples/helperfunc.py
  -> roslaunch data_challenge_simulator load_kuavo_mujoco_sim1.launch
  -> examples/task1.py
  -> task_result.txt
  -> keep successful rosbag / delete failed rosbag
```

## Build

The runtime image archive and the source repository are separate deliverables.
Load the delivered Docker image once, then clone or update this repository and
run the following commands from its root:

```bash
git checkout zjy_openlet
./run-scene1.sh setup
./run-scene1.sh build
```

`setup` mounts the current checkout at `/root/kuavo_ws`; `build` compiles the
shared packages used by all three tasks.  The existing runtime image archive
does not need to be rebuilt for source-only updates.

## Run

Interactive collection, matching the upstream entry:

```bash
cd src/data_challenge_simulator/examples
python3 helperfunc.py
```

Standalone Task 1 waypoint inspection does not use ROS:

```bash
./run-scene1.sh waypoint
```

The repository-root launchers all call the same `helperfunc.py` orchestration
and differ only by task id:

```bash
./run-scene1.sh task
./run-scene2.sh task
./run-scene3.sh task
```

`collect N`, `stop`, and `status` use the same implementation for all three
tasks.  Scene 2 and Scene 3 deliberately reject `verify`/`replay` because they
do not yet have task-specific bag acceptance tools; they never reuse Scene 1's
acceptance criteria.

## External model entry

Each task also has a third, isolated entry for online model inference:

```bash
./run-scene1.sh model
./run-scene2.sh model
./run-scene3.sh model
```

`model` starts the task simulator, cameras, state publishers and the model
compatibility interface.  It does not start `task1.py`, `task2.py` or
`task3.py`, and it does not start `rosbag record`.

Before publishing ready, `model` performs the same one-time initialization the
accepted task entries perform before their first task motion:

1. draw a random seed for this run and print it (override with
   `--seed N` when invoking `model_entry.py` directly, for reproducing a
   specific round);
2. pass that seed's `initial_base_x/y` to the task launch file;
3. place the task objects from the same saved layout catalogue the collection
   entry uses;
4. retreat the base, raise the arm to the fixed ready posture, and return once
   to the measured random base pose;
5. for Task 3, drive to the first seed-derived object docking pose, publish a
   final zero velocity, and unregister the simulator-side `/cmd_vel`
   publisher. Task 1 and Task 2 finish initialization at the returned
   table-front pose and perform no additional model-entry base motion.

Only then does it publish a latched `/model_simulator/ready=true`, meaning the
scene is initialized, the arm is at the ready posture, and the observation
topics are live. Before ready, the entry also switches the arm to external
control and opens the hand command gate. The initialization controllers stop
publishing afterwards, so the external model owns the effective
`/kuavo_arm_traj`, `/sg100_hand_command`, and `/cmd_vel` commands for the
episode. (`KuavoSDK` may leave idle ROS publishers registered, but this entry
emits no commands through them.) The simulator no longer uses object or
hand-state triggers to start later base routes. Stop it with `Ctrl+C` or the
matching `./run-sceneN.sh stop` command.

Collection and model inference use the same full-hand contract:
`/sg100_hand_command` for the 22 joint-position actions and
`/sg100_hand_state` for the measured hand state. The task gripper controller
publishes the standard command directly, and MuJoCo's Heiman implementation
publishes the standard state directly. There is no `/cb_*` conversion bridge.
In model mode the simulator ignores hand commands until
`/model_simulator/accept_commands=true`; afterwards the external model owns
`/sg100_hand_command` directly.

Arm commands still use the upstream `/kuavo_arm_traj` route.  This entry
serves both direct online inference and the upstream automatic multi-episode
evaluation protocol; see the next section.

## Automatic multi-episode evaluation protocol

`model_entry.py` implements the handshake that
`kuavo_data_challenge`'s `kuavo_deploy/src/eval/sim_auto_test.py` drives:

- `/simulator/reset` (`std_srvs/Trigger`, served here) -- starts the next
  round.  A round's object layout and its base spawn point are paired, and the
  spawn is a roslaunch argument, so a reset does not re-place objects in
  place: it draws a fresh seed, tears the simulator down and starts a new one
  from that seed's `initial_base`.  It answers immediately; the scene work
  happens in the background.
- `/simulator/init` (`std_srvs/Trigger`, served by the harness) -- called back
  once the scene is placed, the arm is at the ready posture and the
  observation topics are live.  It is sent after the reset response, because
  the harness clears its `init_evt` when a reset returns.  A latched
  `/model_simulator/ready=true` is still published at the same moment, so a
  plain `./run-sceneN.sh model` session keeps working with no harness present.
- `/simulator/start` (`std_srvs/Trigger`, served here) -- the harness calls it
  after `env.reset()`.  It waits for the round's scene to finish initializing,
  then starts judging success.
- `/simulator/success` (`std_msgs/Bool`, published here) -- `True` the moment
  the task is complete, which is what ends the harness episode; `False` on
  reset and when an episode is rejected.

The model supervisor is shared by all three tasks and treats one round's
startup as disposable.  A required-topic timeout, roslaunch exit, SDK or task
initialization error, arm handoff error, or failed `/simulator/init` callback
tears down only that simulator round and starts a fresh seed after a short
delay.  If the harness has already returned from `/simulator/reset`, that open
handshake is carried across retries and the first successfully initialized
replacement sends `/simulator/init`; failed attempts are never published as
task success.  A standalone model session still runs without an init service.

Success is judged by the task's own scoring rules, not a second definition:
`utils/episode_success.py` drives the accepted `utils/task_scorer.py` scorer
classes' state sampling and reports success when every full-credit condition
they already encode holds at once (Task 1: three cylinders in the target bin
and the lever past 30 degrees; Task 2: both boxes lifted and on the conveyor
with two chassis arrivals; Task 3: three rings lifted and placed with three
arrivals).  Time penalties do not gate success.  `task_scorer.py` is
unmodified, so the `task`/`collect` entries and their score baseline are
unchanged.

Recorded `collect` scores are separated by task under
`examples/scores/taskN/score_taskN_roundM.{txt,json}`.  Repeating the same
task and round never replaces an earlier result: later pairs receive `_2`,
`_3`, and so on.  Existing score files directly under `examples/scores/` are
left in place.  Non-recording `task` runs retain their existing replace-in-
place path directly under `examples/scores/`.

Model scores do not use the `task`/`collect` `examples/scores/` directory.
Start a named model session with:

```bash
./run-scene1.sh model policy_name
```

Each task keeps only two model-score files under
`examples/model_scores/taskN/`: `score.txt` and `score.json`.  Every completed
episode overwrites the previous pair.  The JSON result still records the model
name, task, session, round, seed and finish reason.  Omitting `MODEL_NAME` uses
`anonymous`.

`examples/task1_v2_initialize.py` holds the model-only Task 1 initialization
stage; the internal `v2` name is retained to identify that implementation
generation.  Regression tests keep its fixed values aligned with the accepted
`task1.py`.  It publishes
arm motion with `control_arm_joint_positions` in a plain loop, the same way the
upstream simulator-side initialization does, and leaves no background
publisher behind.

## Task 2 whole-box hand posture

Task 2 uses its own whole-box hand path instead of the older two-pair
double-gripper posture.  Preview the shaped and gripping endpoints without ROS:

```bash
cd /home/zjy/codex/mujoko
./.conda-env/bin/python \
  kuavo-ros-control/src/data_challenge_simulator/examples/preview_box_gripper.py \
  --closure 0
./.conda-env/bin/python \
  kuavo-ros-control/src/data_challenge_simulator/examples/preview_box_gripper.py \
  --closure 1
```

`closure=0` shapes the hand around the box.  During `0 -> 1`, only
`index_j1` (`-1.0 -> 0.0`) and `thumb_j1` (`1.0 -> 1.8`) move; the remaining
joints hold the shaped posture.

## Task 2 grasp-workspace scan

`tools/scan_task2_grasp_workspace.py` is an offline MuJoCo/SciPy tool.  It does
not start ROS or modify the scene.  It samples the base pose, solves both arms
against the dedicated whole-box hand posture, and rejects candidates
whose approach path collides with the table, either box, or non-permitted robot
geometry.

The earlier Y-rim candidates were validated with the retired Task 2
double-gripper posture and are not valid for this hand path.  Use the following
command to evaluate the same coarse region while developing new wrist and
contact targets:

```bash
cd /home/zjy/codex/mujoko
./.conda-env/bin/python \
  kuavo-ros-control/src/data_challenge_simulator/tools/scan_task2_grasp_workspace.py \
  --grasp-modes rim \
  --face-axes y
```

Use comma-separated grids to refine an area and optionally save every feasible
candidate as JSON:

```bash
./.conda-env/bin/python \
  kuavo-ros-control/src/data_challenge_simulator/tools/scan_task2_grasp_workspace.py \
  --grasp-modes rim \
  --face-axes y \
  --base-x-offsets=-0.80,-0.75,-0.70,-0.65,-0.60 \
  --base-y-offsets=-0.10,-0.05,0,0.05,0.10,0.15,0.20 \
  --output /tmp/task2_grasp_workspace.json
```

Exit status `0` means at least one candidate passed; status `2` means the scan
completed but found none.  A scan candidate is a policy-development result,
not a restriction on other valid competition grasp strategies.

## Task 2 full workflow

With Scene 2 running, the formal entry executes initialization and both box
transfers in one ROS graph, then writes the `task_result.txt` handshake used by
`helperfunc.py`:

```bash
python3 src/data_challenge_simulator/examples/task2.py
```

The lower-level staged commands remain available for isolated diagnosis.

The same collection entry used by Task 1 now accepts Task 2.  For example, a
visible no-record validation round is:

```bash
python3 src/data_challenge_simulator/examples/helperfunc.py \
  --task-id 2 --record 0 --repeat 1
```

## Task 2 B1 to grasp probe

With Scene 2 already running, execute initialization and the staged pick entry
in the same ROS graph:

```bash
python3 src/data_challenge_simulator/examples/task2_initialize.py
python3 src/data_challenge_simulator/examples/task2_pick.py
```

The first command first changes only the two shoulder-pitch joints to raise
the arms, then keeps both hand origins at the raised height while turning the
palms toward the table, and stores the measured returned base pose B1 plus the
initialized arm IK seed on the ROS parameter server.  The second command selects the
nearest measured box-front docking target, moves the chassis only when B1 is
outside that target's configured comfortable region, locks the selected box,
plans all bimanual waypoints from live poses, moves the already oriented hands
directly above the box,
shapes the box grippers there, descends, closes, and
performs a short measured lift probe.  `--dry-run` on `task2_pick.py` performs
selection and logging but does not move the chassis or arms.

The docking, grasp, IK, collision, and lift-confirmation values are explicit in
`config/task2_pick.json`.  Its status is `requires_runtime_validation`; passing
the offline nominal-pose check is not a runtime grasp result.

## Task 3 hollow-cylinder scene

Task 3 is a right-hand internal-expansion transfer.  The source table is
`0.65 m` high, the destination table is `0.85 m` high and `0.85 m` to the
robot's left (`+world Y`).  Both tables span world `X=[0.42, 0.82] m`, and the
annulus center is at the Task 1 reach baseline `X=0.62 m`.  The free annulus has a `45 mm` outer radius,
`33 mm` inner radius, and `60 mm` height.

The smooth annulus mesh is visual-only.  Twenty-four nearly invisible convex
wedges provide the physical collision shell, so the bore remains open in the
MuJoCo 3.0.1 runtime.  The Task 3 follower keeps fingertip collision during
approach and insertion.  Before expansion, publish `true` on
`/mujoco/task3_grasp_enabled`; the follower then disables the three active
fingertip collisions and uses a force-free geometric gap test.  Any two
distinct pads among index, middle, and little in the valid inner-wall band
latch the cylinder's full pose relative to `r_hand_base`.  Publish `false` to
arm release, then contract the fingers.  The saved pose remains latched until
two of the three pads have left the inner-wall band.  At that transition the
ring collision shell is disabled and the ring falls freely from rest.  When
its geometric lowest point reaches the destination tabletop, the follower
clamps it to `z=0.85 m` and zeros its velocity without generating a collision
impulse.  Fingertip collision is restored only after the pads clear the ring;
ring collision remains disabled until an explicit object reset.

Run the static geometry, hand-envelope, and short settling audit without ROS:

```bash
cd /home/zjy/codex/mujoko
./.conda-env/bin/python \
  kuavo-ros-control/src/data_challenge_simulator/tools/audit_task3_scene.py
```

Scan the partial-insertion expansion range at the independently measured
hand/ring candidate pose.  The report also prints the reverse-contraction
interval where two pads have left the inner-wall band and release triggers:

```bash
./.conda-env/bin/python \
  kuavo-ros-control/src/data_challenge_simulator/tools/scan_task3_expansion_range.py
```

Exit status `0` means the compact posture is contact-free and at least one
sample places the required number of distinct pads within the force-free
inner-wall detection gap before counterfactual contact penetration.  Exit
status `2` means no range passed.  The defaults match runtime policy: two
fingers, a `1 mm` geometric gap and zero penetration.

Preview the compact insertion and expanded endpoints:

```bash
./.conda-env/bin/python \
  kuavo-ros-control/src/data_challenge_simulator/examples/preview_internal_gripper.py \
  --expansion 0
./.conda-env/bin/python \
  kuavo-ros-control/src/data_challenge_simulator/examples/preview_internal_gripper.py \
  --expansion 1
```

The ROS/MuJoCo scene entry is:

```bash
roslaunch data_challenge_simulator load_kuavo_mujoco_sim3.launch
```

The formal full-flow action entry mirrors Task 1 and Task 2:

```bash
python3 src/data_challenge_simulator/examples/task3.py
```

It runs preprocessing, source docking, compact insertion, the force-free
two-pad latch, a `0.30 m` vertical lift, chassis translation computed from the
live destination-table pose, destination lowering, two-pad-loss release, and
logical no-impulse settling.  It writes `task_result.txt=success` only after
the ring pose confirms destination-table placement; every failure leaves
`task_result.txt=fail`.

Before runtime, audit the same configured arm, hand, lift, transport, lower,
and release paths against the Scene 3 model:

```bash
./.conda-env/bin/python \
  kuavo-ros-control/src/data_challenge_simulator/tools/audit_task3_full_flow.py
```

The common collection entry also accepts Task 3:

```bash
python3 src/data_challenge_simulator/examples/helperfunc.py \
  --task-id 3 --record 0 --repeat 1
```

The configured flow has passed the offline gate and remains pending its first
complete ROS/MuJoCo execution.
