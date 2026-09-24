# Scene 2 cumulative delivery

This branch contains the accepted Scene 1 baseline plus the accepted Scene 2
two-box workflow. Scene 3 code and assets are intentionally excluded.

## Run one recorded Scene 2 round

Inside the `kuavo-mpc-wbc` container:

```bash
source /root/kuavo_ws/devel/setup.bash
cd /root/kuavo_ws/src/data_challenge_simulator/examples
python3 helperfunc.py --task-id 2 --record 1 --repeat 1 --start-seed 1
```

The accepted reference bag is delivered separately from Git:

```text
data_round_0000001.bag
SHA-256: 68c46e48783224370804bc7ca413a82ad238ae6a62f89a842a2f2829f1aa628f
```

Its recorded duration is 158.045786 seconds. The run completed both
False-to-True-to-False bimanual grasp cycles, enabled the destination
conveyor, and ended with `TASK2 FULL WORKFLOW PASSED: placed_box_count=2`.

## Scope boundary

- Scene 1 remains the accepted prerequisite baseline.
- Scene 2 adds initialization, palm-down ready posture, two-box bimanual
  grasp/lift/release sequencing, ready-pose returns, and first-box conveyor
  motion.
- Later tasks must preserve both accepted workflows and add their behavior in
  separate task entry points and task-scoped runtime paths.
