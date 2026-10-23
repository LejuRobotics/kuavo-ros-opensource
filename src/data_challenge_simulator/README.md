# Data Challenge Simulator — S400062 task scenes

## Current Task 1/3 scoring and seeded scenes (2026-10-22)

Task/collect/model now use the same deterministic generation as Task 2:
seed = config/task2_randomization.json seed_base + round (default 27 + round).
Startup retries retain that seed; a completed model episode advances it on
reset. Fixed layout catalogues remain as historical files and are not read
by these entries. Collect no longer generates a separate temporary catalogue.

Task 1 keeps the robot at (0.015, 0, 0). Three cylinders are sampled inside
the blue bin in reachable right-hand regions with 75 mm centre separation.
The green target bin moves +/-100 mm along world Y. Reachability checks use
the existing arm solver; they do not constitute dynamic task validation.
Task 3 keeps the initial robot at (0, 0, 0), samples rings on the source
table with 160 mm centre separation, and moves the entire destination table
+/-100 mm along world Y (initial robot left/right). Placement slots move with
the destination table. Scripted travel crosses between tables through a
front clearance corridor derived from both tabletop positions. Grasp docking
rejects wheel/base intersections with either table, including 35 mm stopping
clearance, and uses a 0.63 m forward ring offset to retain insertion reach under the existing
base-position tolerance; WBC, torque execution and controller gains are unchanged.
Parameters live in config/task1_randomization.json
and config/task3_randomization.json. Both tasks retain Task 2's seeded pastel
camera headlight, fixed white overhead light and wall textures.

Scoring uses read-only MuJoCo geometry/contact flags on each object's
`/mujoco/<name>/scoring_state`, at 50 Hz; the scorer accumulates them at 10 Hz.
A lift requires observed initial source support, followed by loss of support
and positive bottom clearance (0.5 mm numerical tolerance). Falling off the
source below its surface does not qualify. Task 1 awards 10 per lifted
cylinder, 15 per cylinder entering the green bin volume (bottom crosses the
rim with its centre inside; no whole-object containment or stability wait),
and 25 once the blue bin reaches the conveyor end and stays below 0.01 m/s
linear and 0.05 rad/s angular velocity for 0.3 s. The bin's conveyor stop is
0.45 m, distinct from its 0.60 m safety joint limit. Task 1 has 50 free
seconds, then loses 0.5 point/s, capped at 10.

Task 3 awards 5 per lifted ring, 15 per ring above the destination tabletop
(horizontal centre inside its footprint, bottom above the surface), 10 per
ring supported on that tabletop, and 10 when all three items are complete.
It has 60 free seconds, then loses 1 point/s, capped at 20. The legacy
base_arrivals output field now counts distinct arriving rings, not chassis
entries. Table/bin score regions follow current simulation geometry.

Preparation is excluded from timing. Scores latch once earned per round.
The same atomic score lifecycle remains in all entries. Model start waits
for initial physical observations before accepting model actions. The model
entry only prepares the scene; scripted task motions are not run for it.

Final seeded-scene validation: each task passed five consecutive visible
MuJoCo runs (seeds 28–32), all scoring 100. Task 1 scored execution times
39.445/39.842/38.826/39.105/38.804 s; Task 3 used
49.473/52.924/48.076/48.419/47.599 s (preparation excluded).
Each collect entry passed two rounds with readable bags. Each model entry
passed two start/reset cycles and third-round readiness without inference
actions or false success. Scene parameters matched across all three entries.
ROS teardown now stops the topic relay timer and tolerates publisher closure
only after shutdown has begun; live publishing failures remain errors.

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
3. follow the task's initialization sequence and place objects using the same
   task-specific scene generator as collection. Task 1 immediately performs
   its shared preparation: initialize the thumb and scene, then move the hand
   along one continuous curve to the first cylinder's overhead grasp position;
4. complete the task-specific preparation. Task 1 keeps the base at
   `(0.015, 0, 0)` and does not retreat or return the base;
5. finish at the initialized base pose and release the simulator-side
   `/cmd_vel` publisher. All episode-time navigation, including Task 3 ring
   docking, belongs to the external model.

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

The model supervisor is shared by all three tasks and treats pre-episode
startup as disposable.  A required-topic timeout, roslaunch exit, SDK or task
initialization error, arm handoff error, or failed `/simulator/init` callback
before `/simulator/start` tears down only that startup attempt and starts a
fresh seed after a short delay.  If the harness has already returned from
`/simulator/reset`, that open handshake is carried across retries and the first
successfully initialized replacement sends `/simulator/init`; failed startup
attempts are never scored or published as task success.  A standalone model
session still runs without an init service.

After `/simulator/start` accepts an episode, the supervisor does not silently
relaunch on an infrastructure failure because the harness would still be
running the retired episode.  Observer creation, score sampling/finalization,
or roslaunch failures mark that episode invalid, close model command admission,
discard shutdown scoring, and exit non-zero with an operator-action message.
Normal task failure or timeout remains an evaluated episode and is not treated
as infrastructure failure.

Success is judged by the task's own scoring rules, not a second definition:
`utils/episode_success.py` drives the accepted `utils/task_scorer.py` scorer
classes' state sampling and reports success when every full-credit condition
they already encode holds at once (Task 1: three cylinders in the target bin
and the lever past 30 degrees; Task 2: both boxes lifted, delivered over the
belt and supported on its surface; Task 3: three rings lifted and placed with
three arrivals). Time penalties do not gate success. Task, collect and model
use the same scorer rules and snapshot writer.

Recorded `collect` scores are separated by task under
`examples/scores/taskN/score_taskN_roundM.{txt,json}`.  Repeating the same
task and round never replaces an earlier result: later pairs receive `_2`,
`_3`, and so on.  Existing score files directly under `examples/scores/` are
left in place.  Non-recording `task` runs retain their existing replace-in-
place path directly under `examples/scores/`. A fixed current-round snapshot
is also available at `examples/scores/taskN/score.{txt,json}`. It is created
with zero points before simulator preparation, updated during the round,
and reset to the next round's zero without deleting the current file.
Historical collect results keep their non-overwriting filenames.

Model scores do not use the `task`/`collect` `examples/scores/` directory.
Start a named model session with:

```bash
./run-scene1.sh model policy_name
```

Each model command starts a fresh score session. Under
`examples/model_scores/taskN/`, `score.txt` and `score.json` are initialized
to zero and represent the **current** round, including preparation and live
progress. Reset immediately replaces the previous result with the next
round's zero. `average.txt` and `average.json` start at zero valid rounds and
are updated only after valid completed episodes, once per episode. Startup
attempts and infrastructure-invalid episodes do not enter the average;
normal model failures, timeouts and zero final scores do.

TXT remains a plain integer. JSON includes `total`, `components`, task/round
identity, `updated_at` and `status` (`preparing`, `running`, `completed`, or
`invalid`). Model session/seed identity is present during progress; the existing
`model_run` final record, including finish reason, is attached only at valid
finalization. The external evaluator ignores explicit non-completed snapshots
while retaining compatibility with older results without a status field.
File existence does not mean the episode has finished. Each file is atomically
replaced with host-readable 0644 permissions; JSON is the authoritative
snapshot because two separate files cannot be replaced as one transaction.
Completed snapshots are frozen, preventing a retiring scorer from overwriting
the next round's zero or changing its time penalty after finalization.

Task 2 scoring update (2026-10-20): each box earns 10 for leaving tabletop
support upward, 20 when its geometric centre projects inside the actual belt
footprint regardless of height, and 15 for actual support from the belt's
upper surface. Completing all three milestones for both boxes adds 10.
A lift requires prior observed tabletop support, no remaining tabletop
support, and all box collision geometry above the tabletop; there is no
minimum lift distance or absolute Z threshold. Placement accepts any supported
part of the belt, without requiring the whole box or its centre to be inside,
without a fixed Z band, and without an additional hand-release condition.
Side, underside and conveyor-leg contacts do not count as top-surface support.
Milestones remain credited after later movement and are counted once per box.
The old `base_arrivals`/`base_visits` field names remain compatible but now
mean box arrivals, not chassis parking.

The preparation-excluded allowance is 60 s; overtime costs one point per
second up to 20 points, with total score floored at zero. Geometry and positive
support-contact forces are observed read-only in MuJoCo, published at 50 Hz
as `/mujoco/box_1/scoring_state` and `/mujoco/box_2/scoring_state` (`UInt8`:
lifted=1, over belt=2, supported on belt=4, supported on table=8). Scorers sample
at 10 Hz and publish changed scores, including overtime deductions. The new
topics are excluded from the explicit training rosbag topic list. This update
does not change motion, WBC, tau, collision physics or actuator behavior.
The fixed conveyor XYZ constants still used by the scripted placement guard
are not scoring thresholds.

Before every model roslaunch, the supervisor removes the previous simulator's
`/robot_init_state_param` after its old writers have stopped.  This forces the
new controller to wait for the current seed's launch-time base pose instead of
consuming a parameter retained by the process-wide ROS master.

`examples/task1_v2_initialize.py` holds the model-only Task 1 initialization
stage; the internal `v2` name is retained to identify that implementation
generation.  Regression tests keep its fixed values aligned with the accepted
`task1.py`. The caller owns the arm trajectory publisher, which starts
from measured joints and is stopped before model handover.

## Task 2 seeded scenes (2026-10-20)

Task, collect and model use `utils/task2_randomization.py` and
`config/task2_randomization.json`. The legacy `task2_layout_seeds.json` is
preserved unchanged, but these three Task2 entry points no longer read it.
Task2 collect no longer generates or installs a temporary layout catalogue.

The round starts at 1. Its seed is `seed_base + round`, with default base 27:
rounds 1--5 use seeds 28--32. `./run-scene2.sh task 4` therefore runs round 4
with seed 31 (it does not run four rounds). `collect 5` runs rounds 1--5;
restarting the command repeats the same seeded scenes. Model starts at round
1/seed 28, advances after a started episode is reset, and reuses the same seed
when initialization is retried before an episode starts. A reset before the
first start also keeps seed 28. The direct model-entry `--seed` override sets
the first seed, and later scored rounds increment it by one. Tasks 1/3 retain
their existing seed semantics.

The table stays fixed, and the robot starts at (0, 0, 0), facing the table.
The complete conveyor translates in world X by a uniform [-0.10, +0.10] m;
its Y, height and orientation do not change. Both boxes have fixed orientation
and stand 1 cm inside the near tabletop edge. Their Y positions are sampled
jointly within the actual tabletop footprint, with at least 2 cm to the outer
edges and at least 12 cm between box walls. This conservative gap includes
room for the existing open-hand approach; it is not a claim that one finger
is 12 cm wide. The remaining free space is distributed randomly, so the two
positions vary without overlap and without using a finite catalogue.

A temporary XML beside the original supplies the identical geometry to the
simulator and local task initialization/IK planners, and is removed on exit.
Source XML/assets remain unchanged. Boxes are initialized through the existing
object-placement service. The generated scene parameters are printed at startup.
Task/collect docking uses live box positions; the placement target accounts for
the shifted belt and held-box width. Placement confirmation uses actual belt-top
support, not the retired fixed-coordinate check. WBC, torque transmission,
actuators, collision properties and task/model command ownership are unchanged.

Task2 keeps the ICRA `scene2.xml` scene light fixed: white, directional at
`(0, 0, 7)`, facing `(0, 0, -1)`, with ambient `0`, diffuse `0.7`, and specular
`0.3`. Only the camera headlight color is randomized, choosing a bright pastel:
light red `(0.9, 0.7, 0.7)`, light green `(0.7, 0.9, 0.7)`, or light blue
`(0.7, 0.7, 0.9)`. Neutral headlight ambient `(0.25, 0.25, 0.25)` remains fixed
to keep shaded boxes and grasp edges visible; specular is zero. A separate
seeded random stream makes lighting reproducible without changing layout draws.
This deliberately replaces ICRA's generic diffuse-jitter rule and negative
ambient example, which can produce dark colors and obscure shaded objects.
The MuJoCo headlight follows the viewing camera; it is not a physical robot lamp.
The generated XML is shared by GUI and policy cameras, and its marker prevents
the old C++ headlight ranges from overriding it. Every startup prints
`[TaskLighting] ... source=scene_xml` with the actual light values.
Tasks 1/3 retain their previous lighting behavior.

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
`index_j1` (`-1.0 -> 0.0`) and `thumb_j1` (`1.0 -> 2.1`) move; the remaining
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
outside that target's configured comfortable region, then
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
MuJoCo 3.0.1 runtime. Task 1 cylinders and Task 3 rings now keep their
collisions enabled throughout grasp, transport, and release. The simulator
observes force-bearing finger contacts; it does not freeze finger joints,
attach object poses to the hand, or force objects upright after release.
The compatibility topics `internal_grasped` and `internal_finger_latch_mask`
report current contacts, not a persistent attachment. The new per-object
`destination_supported` topic reports upward force-bearing contact with the
destination surface. Task scripts validate actual object lift and placement.
Task 1's lever collision radius matches its 14 mm visual radius; the previous
18 mm hidden shell stopped the hand 4 mm short of the visible surface.
The grasp point uses the solid palm ridge instead of the recessed region
bridged by its convex collision hull. A mesh-ray regression checks the visible
surface at the planned contact, in addition to collision contact tests.
Its lever sequence first seats the palm face on the rod, then closes
fingers around it. The `/mujoco/lever/palm_seated` event requires 50 ms of
force-bearing palm-face contact and does not unlock the hinge. Unlocking
requires the palm and at least two fingers to exert opposing contact forces
for 50 ms of actual contact. Solver force gaps of up to 10 ms pause that
timer; longer gaps reset it. `/mujoco/lever/grasp_confirmed` reports the
confirmation event. The live
`grasp_contacts` mask continues reporting contacts during lifting. No rod
attachment, imposed pose, or extra lifting force is used.

Task 1 and Task 3 use the same seeded pastel headlight and fixed white
overhead light as Task 2, including the same `wall3.png` wall material.
The historical validation below predates the seeded scenes and scoring
revision documented above. Task 3's baseline near-edge slots are at world XY
`(0.52, 1.04)`, `(0.52, 0.90)`, `(0.52, 0.76)` m, filling far to near.
This clears the destination table legs and previously placed rings while
preserving the existing arrival and placement score regions. These are action
targets, not changes to the initial layout catalogue.

Task 1's visible cylinders retain their radius, height and mass. A convex
32-sided mesh replaces each primitive collision cylinder (maximum radial
difference 0.126 mm), and multi-point collision detection is enabled. This
avoids incorrect cylinder/box corner normals that trapped released objects
inside the target floor or side walls. A 30-drop randomized orientation test
requires final penetration below 1 mm. Actual task validation also checks
the released object's full extent against the bin and records penetration,
rather than relying only on its centre or score. Task 1 disables the optional
no-slip post-processing, which otherwise locks simultaneous floor/wall
contacts in penetration. Ordinary friction and collision response remain
active. Regression cases cover both the original near-wall source spawn and
a real tilted release that previously remained embedded 3.23 mm into the bin.
Spawn positions and randomization parameters are unchanged.

The source-bin floor and all four walls explicitly use six-dimensional
contact with friction `1.2 0.02 0.001` (sliding, torsional, rolling). This
raises only the interior torsional coefficient from 0.01 to 0.02; the
object/finger coefficients and free-joint damping are unchanged. A dynamic
regression removes joint damping, spins the supported cylinder at 20 rad/s
and checks decay below 0.05 rad/s within 0.5 seconds, both at the centre and
near a wall. This resists residual spin after an impact; it does not lock an
object against a hand that keeps applying torque.

Task 1 keeps its base stationary during picking, placement and lever lifting.
The robot-side target conveyor rail and its matching legs are 35 mm farther
forward, keeping the initial thumb clear during elbow settling. The bin and
cylinder poses are unchanged, and the frame keeps its collision geometry.
Previously the nominal straight-arm clearance was only 3.25 mm: a 0.01 rad
elbow bend hit the rail and displaced the wrist before preparation started.
Its preparation uses one four-second joint-space Bezier curve with a quintic
time law, ending 0.85 m high directly above the first cylinder. The clearance
control point is not a stop or a commanded waypoint. Task and model use the
same trajectory builder. The hand approaches each cylinder from above, lifts the grasp to 0.81 m, and
carries at the achieved height. Above the green bin the palm faces down,
with a horizontal target orientation; it releases there without lowering or
withdrawing. Fingers first open enough to let the cylinder fall, then fully
open after settling. Before lifting the lever, fingers hold an open pregrasp
posture that clears the blue bin wall. The palm approaches the rod, seats
against it, and only then do the fingers curl around the rod. During closure,
small palm-normal corrections restore lost contact before fingers close
further. Fingers close slowly through a small preload after first contact,
with distal targets capped at their joint limit. The lifting arc starts only
after a confirmed physical grip, from
the command endpoint so the grip is not unloaded by a reset to measured joints.
Lift and transfer now share one preplanned trajectory with a 15 mm rounded
corner at the high carry waypoint. The original combined 130 command samples
(2.6 s) are retained; the arm does not stop for a second IK solve or restart a
zero-velocity profile at the corner. A measured 80 mm object-lift check runs
at the high-clearance transition before continuing the transfer. The object
in-hand offset is measured after finger closure for this combined plan.
Lever approach and descent similarly share 155 samples (3.1 s), passing
through the same overhead waypoint with a 15 mm rounded corner.

Cylinder closure retains its verified contact-based preload and a 0.2 s
post-preload contact check before lifting. Release keeps the
verified 0.5 s partial-open and 0.3 s full-open movements, then checks entry
into the target region and immediately proceeds. It does not wait for the
object to stop moving or maintain contact with the floor.
These changes affect task/collect actions and model
preparation; inference actions still belong to the external model. Source
closure slows after first finger contact and uses a small preload instead of
continuing to squeeze at approach speed. Shorter approach, transfer and
release timings retain the same grasp feedback, high carry path, physical
lift checks and bin containment checks. WBC, torque delivery, object
randomization and scoring are unchanged.

Preparation-only verification on 2026-10-22: recorded visible runs covering
seeds 1--5 took 4.066--4.078 seconds for the arm preparation curve. Measured
startup wrist peaks were 0.105--0.140 degrees, versus 18.868 degrees before
the rail clearance fix. Replaying measured arm/hand states found no
right-arm/fixture contacts, including the convex collision shells. Endpoints
were 0.1643--0.1647 m above the first cylinder with 0.0162--0.0165 m horizontal
offset. The shared model preparation was also exercised to its ready state;
this is not a model inference test. The 57 targeted regressions passed.
These are preparation checks across recorded runs, not five consecutive
successful full tasks. The subsequent palm-seating repair and full-flow
validation are recorded separately below.
Preparation reports and the before/after measurements are saved under the
evidence directory below (`*-preparation.json` and
`preparation-clearance-comparison.json`).

Visible Task 3 validation on 2026-10-21: seeds 1--5 passed consecutively,
all scoring 100, with motion times after preparation of 44.449, 43.217,
44.852, 42.536 and 43.950 seconds. Every ring passed physical lift, released
finger contact and destination-support checks.

Visible Task 1 validation on 2026-10-22 (`task1-no-settle-final-b`, seeds 1--5)
completed consecutively with 100 points in every round and no runtime errors.
Object extent and penetration checks passed. Times excluding preparation,
including cleanup, were 39.494, 39.034, 39.976, 39.241 and 39.088 seconds.
The 64 targeted regressions passed. Motion recordings show the former
0.19--0.23 s lift/transfer planning gaps replaced by continuous joint motion.
Two faster candidates failed grasp or placement and were reverted; the
30-second target is not achieved. A separate palm-surface audit still finds
visible rod/palm gaps in some runs despite successful physical grasp and
lifting. Moving the grasp farther along the palm failed the geometry check
and was not retained, so these passes do not certify a gap-free lever grasp.

Evidence is under `/home/zhangyutao/kuavo-workspace/task13-repair-20261021/`
(`task3-final-b-seed*`). Per-run task and ROS logs are retained. The ROS master
can log `publisherUpdate` faults or connection refusal after nodes shut down;
these shutdown notifications are recorded separately with timestamps. No
such exception is accepted during task execution, and all other errors remain
validation failures.

The visible MuJoCo window now also opens when a remote X desktop has no
connected physical monitor, using a 1280x720 fallback instead of asserting
inside GLFW. This does not change simulation dynamics.

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


## Task 2 physical grasp validation

Run ./run-scene2.sh task 1 from the repository root after rebuilding
mujoco_cpp. Both picks now use the standard arm and SG100 joint commands.
The task requires physical contacts on both hands, at least 5 cm measured
box lift, a 5 second hold, and release inside the destination conveyor bounds.
A latch flag or a low box height outside the conveyor is not a successful pick
or placement.

Task 2 no longer subscribes to task2_grasp_enabled or
task2_reposition_enabled, disables grasp collisions, freezes fingers/arms,
or attaches a box to the hand pose. The bimanual_grasped topic is a read-only
physical contact observation; it does not constrain motion. After release,
the conveyor applies a bounded drive force only while the box has physical
belt contact. It never overwrites the box pose or velocity. This is a
simplified contact-driven belt model: its finite drive also overcomes the
friction of the stationary belt geom; it is not a moving-surface solver.

The scene uses MuJoCo friction solving (impratio=10, noslip_iterations=10).
These are contact-solver settings, not object attachments. Like the ICRA
simulator, arm actuators execute the controller torque without an additional
simulator-side joint-position/velocity PD loop. The original XML motor
control limits remain in effect. Incoming joint-command torque is copied
unchanged; the simulator-side inertia, damping and friction compensation
code and its configuration have been removed. The upstream controller is
unchanged.

The robot now has 50 additional zero-mass convex collision shells reusing
its visual meshes, including the palms, finger segments, arms and torso.
Their collision bit 8 is accepted only by Task 2 objects and static geometry;
Task 1/3 collision masks and robot mass/inertia are unchanged. Boxes keep
their floor and four wall colliders. Each box now weighs 1.0 kg (previously
5.1552 kg), with proportionally reduced inertia. Contact stiffness for the
new shells and boxes uses solref="0.01 1", solimp="0.95 0.99 0.001".
Hand motor gains, friction coefficients and robot actuator settings are unchanged.

The Task 2 example places the fixed jaw at 0.228 m in the box frame, grips
0.055 m ahead of its centre and closes the thumb to 2.1 rad within the existing
2.182 rad limit. The fixed-jaw contact height is 0.075 m above the box
origin, keeping the grip below the rim. Both hands follow one 0.20 m vertical
target path with 120 trajectory points. Actual lift and five-second holding are checked separately.
The example docks 0.68 m from the box centre (previously 0.65 m) to
clear the table legs during rotation. Docking checks both longitudinal and
lateral error before skipping chassis alignment. Boxes have a 0.10 m minimum
wall-to-wall gap: the generator's box_2 Y range is now [0.12, 0.30] m.
Only saved layouts 4, 6, 8 and 9 needed their box_2 Y and corresponding
docking Y raised to 0.12 m; other saved positions are unchanged. This also
changes the scene distribution used by collection and evaluation. The
old closest gap of about 0.019 m let a hand catch the neighbouring box.
The task now rejects a grasp that also raises the unselected box by the
minimum accepted lift height.
The extra post-retreat lift is removed: with the lighter box it tilted the
load instead of improving clearance. After picking, the robot backs away
from the table to X<=0.04 m before turning in place. This leaves room for the
chassis and wheel sweep even with the 0.03 m stopping tolerance; turning from
the grasp pose could jam the base against a table leg. It then goes directly
to the right side of the conveyor, retaining the actual turning X and
targeting Y=0.30 m. It does not slide sideways to the old fixed destination
X=-0.20 m. For these translation segments the
task's speed command preserves the direction toward the goal when limiting
speed, instead of clipping each axis independently; the same /cmd_vel
interface and downstream wheel controller are retained. After the first release it resets the arms,
backs straight out to Y=0.10 m along the arrival line, then turns back and goes
directly to the other box's docking pose. The short retreat clears the table
and conveyor legs from the chassis/wheel turning sweep; turning directly at
Y=0.30 m could jam the chassis against the table leg. It no longer
retreats 0.70 m or returns via the stored preparation point B1. The second
release ends the task at the conveyor. Collection and policy evaluation must
use the same scene physics settings; old assisted-grasp recordings do not
validate physical grasping.

Task 2 scoring accepts the whole belt footprint, X in [-0.48, 0.40] m
and Y in [0.62, 2.42] m, without a left/right placement target. Each distinct
box entering that footprint above belt height earns its 20 transport points;
entering the existing placement-height band earns its 15 placement points.
The fixed chassis arrival point has been removed. The report fields
base_visits/base_arrivals are retained for compatibility and now count unique
box deliveries, preventing repeated credit for the same box. Grasp points,
completion bonus and time penalty are unchanged; total remains 100.
Scoring remains pose-based, not a direct contact-force or hand-release detector.
The task script separately checks hand opening and release.

Repeated-run startup and teardown now distinguish wheel-arm SDK requirements
from biped-only COM/gait topics. Task 2 waits for the first WBC observation
before initializing SDK command clients, rather than treating early simulator
sensor publication as controller readiness. Unconnected optional arm interfaces
are reported as warnings; required interfaces retain their failure checks.
MuJoCo initialization and camera snapshots
use the same simulation-data mutex, preventing concurrent mj_forward/mj_copyData.
MuJoCo nodelet unload requests simulation stop and joins its thread, restores
the previous SIGINT handler, and stops the hand worker before model disposal.
The UI is destroyed on its owning thread before GLFW termination, avoiding
the exit-time GLFW TLS assertion.
The shutdown guard lets roslaunch shut down the shared nodelet process normally,
rather than removing the sensor simulator while the controller is still running.
MuJoCo keeps the manager's ROS SIGINT handler in nodelet mode; its standalone
executable retains its own handler. Scorer termination unregisters its ROS node.
Waiting for the MPC reset service is reported as startup progress; actual
reset RPC failures or a negative completion result remain errors. These
changes do not alter WBC, torque computation or actuator control parameters.

Current Task 2 timing and approach update (2026-10-20): the ready hand
origins target base-frame X/Y=0.54/+/-0.20 m and world Z=0.98 m. The contact
approach starts 0.16 m above the grasp and follows the reversed Cartesian lift
path downward, without a separate retract stage. Offline checks require hand
collision geometry to clear the box rim by at least 0.05 m and constrain the
sampled descent's horizontal deviation to less than 0.003 m. Actual controller
tracking still has small alignment corrections; these are not an exact
zero-horizontal-motion guarantee.

Task/collect motion limits are now 0.60 m/s and 1.20 rad/s, with the existing
0.08 minimum speeds and arrival tolerances. Above/descent/lift commands use
20/35/60 points at 0.02 s, release-ready uses 30 points, and gripper motion
uses 0.35 s. Fixed lift-settle and timed hold settings have been removed.
Fresh measured hand poses must reach their waypoint and settle before closing
or transporting; actual box rise and two-hand contact are still required.
A missed hand target or missing observation fails the task rather than silently
continuing. Task 2 may finish the existing 0.6 s zero-command stop early only
after new measured velocity samples confirm stopping; target tolerances are
then rechecked as before. Other tasks retain their default stop behavior.

Opening the grippers overlaps the high approach; already-open hands skip the
redundant opening motion. After confirmed release and conveyor contact, empty
arm reset overlaps the original straight retreat. Both finish before the return
turn and conveyor activation, and worker errors propagate to the task. Required
physical settling/contact checks remain, so this removes idle holds rather
than promising uninterrupted nonzero robot velocity. WBC, torque calculation,
actuator behavior, scene physics, collision geometry and scoring were not
changed by this timing update. Model actions remain externally generated;
model initialization shares the raised ready posture.

Five consecutive fresh `./run-scene2.sh task <seed>` launches, seeds 5, 3, 4,
1, 2, completed in 34.556, 33.202, 32.400, 34.816, 30.890 seconds excluding
preparation (mean 33.173 s). Timing still spans the first
pick/transport stage start through the second stage completion, including
script docking, IK and final stopping. All five scored 100 without errors,
exceptions or crashes in the collected terminal/ROS/internal logs. Full launches
including preparation and cleanup took 77.57--81.56 s.
All ten loaded transports retained both-hand contact in every 20 Hz sample;
minimum box origin Z was 0.7509 m and maximum tilt 7.83 degrees.
The 55 targeted regression checks passed. Runtime/configuration SHA-256 hashes
were unchanged across the five runs. Evidence, failed experiments, backups and
final snapshots are under
`/home/zhangyutao/kuavo-workspace/task2-speed35-20261020/`.
These five saved layouts do not certify every fresh collect layout or model
policy, and the measured worst-case margin below 35 s is 0.184 s.

Earlier Task 2 parameter-only tuning on 2026-10-20 (superseded above) changed only `config/task2_pick.json`
execution parameters: linear/angular speed limits 0.40 m/s and 0.90 rad/s;
above/descent/lift trajectories 35/50/90 points at the existing 0.02 s step;
post-release ready trajectory 40 points; gripper duration 0.5 s; loaded hold
1.0 s instead of 5.0 s. Minimum chassis speeds, stopping tolerances, physical
contact/release checks, 0.6 s lift settling, retreat distances, action order,
IK geometry, WBC, torque computation and actuator control remain unchanged.
Task and collect share these parameters; model episode actions remain external.

With this same configuration, five consecutive `./run-scene2.sh task <seed>`
runs (seeds 5, 3, 4, 1, 2) completed both boxes in 44.562, 44.010, 43.269,
43.822 and 41.531 seconds, respectively, excluding preparation. Timing is from
`pick_transport_release_1/2` start to `pick_transport_release_2/2` completion,
including task-script docking, IK and final stopping. Each scored 100 with no
ERROR/FATAL, exception, crash or roslaunch signal escalation in the collected
terminal/ROS/internal logs. Full launches including preparation and cleanup
took 88.05--91.19 seconds. Read-only 20 Hz pose/contact sampling over all ten
loaded transports observed both-hand contact at every sample, minimum box
origin Z=0.7531 m and maximum box tilt 8.03 degrees; no drop was observed.
Seventeen targeted time-optimization, reachability and physical-grasp tests
passed. The evidence and pre-change parameter backup are under
`/home/zhangyutao/kuavo-workspace/task2-speed-20261020/`.
These five task layouts do not certify all fresh collect layouts or model
policies. Existing SDK/URDF startup warnings are not suppressed by this tuning.

Before speed tuning, repeated-run validation on 2026-10-20 used five consecutive fresh launches of
`./run-scene2.sh task <seed>`, in order 5, 3, 4, 1, 2, all scored 100 and
completed two physical picks, five-second holds and right-side placements.
Elapsed times including startup/cleanup were 115.78, 114.84, 116.58, 115.25
and 113.23 seconds respectively. Terminal, per-run ROS and internal simulator
logs contained no ERROR/FATAL, traceback, assertion, abort or segmentation
fault; roslaunch required no signal escalation. Observed box rises were
0.149--0.179 m; the other box was not lifted. Released box centres were
X=0.038--0.084 m and Z=0.560 m, on the right half of the belt.
The final regression run passed 220 tests, excluding only the two separate
collection-directory suites (`test_collect_bag_directory.py` and
`test_collect_bag_mount.py`).
Evidence is retained on the server at
`/home/zhangyutao/kuavo-workspace/task2-validation-20261020/`.
The five-round pass is specific to these layouts; it does not certify every
random layout or learned policy.

Earlier right-side placement validation on 2026-10-19 (MuJoCo 3.0.1):
`./run-scene2.sh task 1` and `./run-scene2.sh task 2` both completed
two physical picks and right-side placements, scoring 100 in 108.22 s and
106.22 s respectively. Final released box centres had X=0.075--0.096 m,
inside the right half of the belt. Loaded translations retained the source
docking X instead of moving sideways to X=-0.20 m.

ROS pose records showed approximately 0.18 m lift, maximum carried pitch
3.35 degrees and roll 1.16 degrees after lift settling. During the initial
lift, pitch briefly reached 8.42 degrees; this is not an all-phase
plus/minus-five-degree guarantee. Released boxes settled at Z=0.56 m,
within 0.01 degrees of level. A reconstruction from measured robot/hand/box
poses found peak robot-box contact penetration of 2.63 mm; asynchronous
observations make this an estimate, not an exact solver contact trace.
Dedicated force-push tests verified physical blocking by the added shells,
and the production conveyor-force test verified that an obstacle stops
transport without position/velocity overwrites.

Regression: 202 passed, excluding the two pre-existing collect-bag directory
and mount tests. Run the suite separately from simulation because its
shutdown-guard test can terminate a running simulator.
Logs: `/tmp/task2-right-final-seed1.log`,
`/tmp/task2-right-final-seed2.log`,
`/tmp/task2-right-final-regression.log`.
Pose-derived summaries: `/tmp/task2-right-final-seed1-metrics.json` and
`/tmp/task2-right-final-seed2-metrics.json` on the host. These results validate
the scripted task, not trained-policy performance.

The results below describe earlier versions, before the collision, mass,
conveyor and direct-route changes above.

Earlier runtime validation on 2026-10-19 used MuJoCo 3.0.1 with the
additional arm PD enabled. Seeds 1 and 2 both scored 100. Those results
do not validate the torque-only execution path or a trained policy.


Torque-only validation on 2026-10-19 rebuilt mujoco_cpp and passed 184
simulator tests (excluding the two existing collect-bag test files).
The 14 arm actuators retain their original torque limits and have no added
affine PD bias. Both `./run-scene2.sh task 1` and `./run-scene2.sh task 2`
scored 75 and failed the complete task: both boxes rose approximately 6.9 cm
and passed the 5 second hold, but only box_2 was placed successfully.
For box_1, the release check timed out because the box was below and outside
the destination conveyor (last z: 0.154 m / 0.181 m; required contact height:
at most 0.605 m, with position inside the conveyor bounds).
These runs confirm physical lifting without the additional PD or grasp
attachment; they do not establish reliable transport and placement.
Logs: `/tmp/task2-tau-only-seed1.log`, `/tmp/task2-tau-only-seed2.log`.


Earlier task-level transport repair, validated on 2026-10-19 after removing all
simulator-side arm torque compensation: the former loaded path let a tilted
box corner catch the conveyor edge. That version gripped 0.04 m ahead of
the box centre (formerly 0.10 m), lifted the hands by 0.20 m with 120 trajectory
points, retreated 0.70 m clear of the source table, then used the existing IK
planner to lift another 0.12 m before turning at 0.25 rad/s. That second lift
compensated for an initial lift pivoting around an edge still on the table.
This route and second lift are no longer used. The existing docking poses, destination base
pose, hand-open checks, conveyor-contact checks and scorer are unchanged.
No WBC, actuator, gain, torque-processing, collision, mass, friction or grasp
attachment changes were made for this transport repair.

Both visible validation commands completed two physical picks and placements:
`./run-scene2.sh task 1` scored 100 in 132.04 seconds, and
`./run-scene2.sh task 2` scored 100 in 130.96 seconds (whole round timings).
A read-only pose/contact observer confirmed the first run's box lower corners
at 0.681 m / 0.695 m immediately before release, above the conveyor edge at
0.60 m. Both boxes settled flat at approximately z=0.56 m with hand contact
released. Conveyor movement after placement remains the pre-existing scripted
transport; this validation does not claim that transport is physically driven.
These are scripted-task results, not trained-policy evaluation.

Runtime logs: `/tmp/task2-transit-lift-seed1.log` and
`/tmp/task2-transit-lift-seed2.log`. Regression result: 185 passed, excluding
the two pre-existing collect-bag test files; log:
`/tmp/task2-transport-regression.log`. Run the regression suite separately
from simulation: the existing stale-process cleanup test can terminate a live
simulator. An initial seed-2 launch was interrupted by that concurrent test;
the successful seed-2 result above is the isolated rerun.


### Task 1/3 score lifecycle verification (2026-10-22)

The model entry now waits for arm-controller services before constructing the
SDK, avoiding startup arm-mode query timeouts. The observation compatibility
publisher stops and joins both publishing timers in its client shutdown hook,
before ROS closes publisher topics; this fixes the timer callback exception
during model scene resets.

A fresh Task 3 collection layout exposed an unreachable 35 mm insertion from
an otherwise accepted parking pose. Insertion is now 27 mm. Both recorded
edge layouts completed with physical grasp/release checks and 100 points.
Tighter parking tolerances/speeds and a forward docking shift were tried and
reverted; chassis motion, WBC, torque delivery, object layout generation and
scoring rules retain their previous behavior.

Visible verification covered Task 1 task (100), Task 1 collect (two rounds,
100/100), Task 3 collect (two fresh rounds, 100/100), and both model entries
(two zero-action episodes and readiness after the second reset). Model checks
exercise the actual simulator protocol, current-score reset and session
averaging; they do not validate learned-policy actions. Both pairs of collected
bags were finalized and readable. Score snapshots were continuously read during
preparation, scoring and round changes without parse or permission failures.
Invalid-round exclusion and stale-observer protection have Task 1/3 regression
coverage.

The final unchanged Task 3 implementation passed five consecutive visible
task launches (seeds 1--5), all 100 points with no runtime exceptions. Motion
times excluding preparation were 45.393, 43.767, 45.180, 43.539 and 44.653 s.
This does not meet the earlier 35 s target. The 93 targeted regressions passed.
Nine preexisting checks in the separate external bag directory/mount suites
still fail because that external-storage feature is absent; those checks were
not counted as passing. Specific ROS startup/shutdown connection notices remain
preserved separately from runtime failures.

Evidence and full snapshot/log audits:
`/home/zhangyutao/kuavo-workspace/task13-lifecycle-20261022/final-report.json`.
