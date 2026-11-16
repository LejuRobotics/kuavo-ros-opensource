# Absolute velocity IK filter alignment

Base: beta `1818fe2074d824ba46c5c02110fdf7bb96a48c05`.
Filter/control reference: test `7ea9a70d06d8dc744d6a4e3ec427d7b71523a6f0`.
At final verification, remote test advanced to `04e044340df8d23cf0849b93230311e0cdcdc98e`
(lower-body TUI support). Its filter configuration and ordinary tracking
parameters are unchanged; the new TUI feature is outside this alignment.
All changes are inside `src/wheel_arm_abs_vel`. This is an offline candidate,
not a robot deployment or a real-robot motion acceptance.

## Resulting default behavior

The private Mapper filters the source skeleton before TCP geometry mapping.
Position CV uses sigma=0.001 and q=0.63; rotation CV uses sigma=0.003,
chest q=5.7 and palm q=1.42. Kalman velocity correction is unscaled (gain=1).
The existing native numerical kernels and native geometric retargeting remain
in use. The mapped TCP, elbow and body derivatives share the source epoch.
The old hand-only CV and extra elbow geometry smoother are not used by this
production path.

The IK shapes observed TCP position/orientation at 5/4 Hz and elbow position
at 5 Hz. Feedforward is the derivative of the shaped reference. There is no
hand source-age extrapolation. Torso shaping remains 2.5 Hz.

| Parameter | Default |
| --- | --- |
| TCP position gain / weight | 12 / 12 |
| TCP orientation gain / weight | 8 / 6 |
| Elbow position weight | 0.15 |
| Acquisition linear / angular correction speed | 0.4 m/s / 1.2 rad/s |
| Left/right J4 jerk | 25000 rad/s³ |
| RNEA measured-velocity cutoff | 30 Hz |
| RNEA arm dynamic-feedforward jerk | 500 rad/s³ after acceleration gain |

S6 J3/J5/J6/J7 caps remain unchanged. Both RNEA configuration selections use
the test baseline, including the 0.05 s sensor timeout and maximum period.
`hand_prediction_config` is retained as an optional explicit parameter overlay,
but defaults to empty. The historical `layer2_causal_candidate.yaml` is no
longer loaded by default and cannot re-enable the removed predictor.

## Preserved beta behavior

- The first controller follow requires both front triggers and successful
  confirmation. Later starts require X+A. X+B only pauses/resumes an existing
  active arm session. Optical OK admission remains unchanged.
- Session binding, execution leases, owned-reference retention, PREPARE and
  torso/arm permission transitions remain beta's implementation.
- QP result acceptance, fixed-size SHM publication, asynchronous diagnostics,
  private recording topics, model contracts and CPU scheduling remain intact.
- Shared packages, incremental IK, hand drivers and end-effector gains are
  outside this change.

While exercising the real Mapper method, the missing local chest-geometry
initialization in `compute_hand_pose` was restored using beta's existing
`_chest_frame_geometry` helper. Deferred observations/markers remain intact.

## Verification

The package is built in an isolated Ubuntu 20.04/ROS Noetic container on the
yyb workstation, with cached dependencies and a separate build/devel directory.
No robot devices are mapped; no controller, simulator or hardware driver runs.
Launch tests parse source resources from the candidate rather than stale
installed resources in the dependency underlay.

- Frozen test-branch data covers changing 72/90/120 Hz sampling, duplicate
  timestamps, quaternion sign changes, source changes, a long gap and session
  reset. The retained native path is compared with test's Python skeleton
  filter and TCP geometry, with 2e-7 absolute/relative tolerance.
- Packet-to-TCP regression executes the production Mapper methods and checks
  rejected input, optional root data and optical input without a joystick.
- Production IK shaping is exercised with static, ramp, stop, reversal and
  sinusoidal inputs at 72/120 Hz and variable packet timing for models 62, 63
  and 400062. Checks include configured gains/J4 limits, derivative consistency,
  settling, mean ramp slope, and independence from observation age and raw
  feedforward. Sampled-input ripple is not mistaken for a constant instantaneous
  500 Hz velocity.
- Quest/Pico launch tests check the final parameters after all YAML overlays.
- Existing startup, lifecycle, QP, SHM, recording/model and RNEA tests are retained.

Final validation: package Release build and test targets passed; **80/80 CTest**
entries passed. The 360-frame packet-to-TCP comparison against 19 frozen
test-branch snapshots had maximum absolute difference `2.30e-15`.

This establishes offline code/parameter alignment. Live latency, tracking
quality, hardware timing and robot motion still require separate acceptance.
