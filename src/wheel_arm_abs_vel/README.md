# Wheel-arm absolute velocity IK

This additive package contains the S62 absolute teleoperation mode. Its source
was extracted from a145c974e3b475d0740acd24cd36346b1723573b onto official dev
9ca0b7b880188faa1b686b48e906c032c36124d7. No existing repository file is changed.

## Selection

Existing launch commands keep their existing behavior. The new selector defaults
to the original wheel controller:

    roslaunch wheel_arm_abs_vel control.launch

Select the new controller explicitly:

    roslaunch wheel_arm_abs_vel control.launch controller_type:=wheel_arm_abs_vel

The direct load_kuavo_real_velocity_ik.launch and load_kuavo_mujoco_velocity_ik.launch
entries also explicitly select this mode. The new plugin rejects a missing
selector, a non-wheel controller, an incompatible estimator, or external MPC.
Only S62 has a private model contract.

The controller and VR input are separate processes. Input entries are
wheel_arm_abs_vel/quest_velocity_ik.launch and wheel_arm_abs_vel/pico_velocity_ik.launch.
Keep the existing input IP and robot settings appropriate to the deployment;
these commands are not hardware qualification. Do not run legacy and absolute
command writers together.

The real:=false option selects the existing visible MuJoCo infrastructure and
the WBC backend. Real mode defaults to RNEA. RNEA is not the default for the
existing torque-only MuJoCo actuator model. Backend selection is startup-only.

## Isolation and shared safety

- Controller/nodelet, Quest FSM, Mapper, Pinocchio/ProxQP solver, reference owner,
  messages, configs, and scheduling helpers live under this package.
- Eight new ROS messages use the wheel_arm_abs_vel package namespace. Existing
  messages, services, protobuf fields, raw VR packets, and kuavo_msgs/jointCmd
  are unchanged. Old launch graphs never load the new plugin.
- HardwareNodelet is the original dev plugin. Its command conversion,
  cmdTruncation, JointProtectionAction, actuator drivers, stop handling, and
  hardware startup remain authoritative. The new launch explicitly enables the
  original cmd_truncation_enable switch; the legacy launch is unchanged.
- AbsoluteWbcBase derives from the original WbcBase. Torque constraints,
  lower-body and arm-error protection, and task settings call the original
  implementation; the new dynamics/reference terms are separate.
- A new controller instance latches measured posture with zero desired velocity
  before ownership. The reference manager admits fresh, valid atomic q/dq/ddq
  transactions and owns bounded stopping after source/lease loss. Release retains
  the stopped endpoint, without re-entering a legacy trajectory writer.
- Stop and controller teardown notify the existing hardware lifecycle. This
  package adds no motor driver or copy of the hardware safety layer.
- The S62 full-model reduction and guarded IK intervals are in
  config/model_s62.json; baseline assets and URDFs are unchanged.

The raw donor's shared RT changes and copied AbsoluteHardwareNode are excluded.
Consequently deferred hardware initialization and donor-only hardware telemetry
are not part of this refactor. Real hardware follows official dev startup.
No claim of identical hardware timing or physical qualification is implied.

## Build and validation

Build in the project's provisioned remote ROS Noetic environment with the same
Drake, Pinocchio and runtime libraries used by the existing controllers. The new
package uses Catkin and contains its own bundled ProxSuite headers.

Do not blindly execute repository-wide ALL targets: the unchanged kuavo_assets
ALL target includes installation/configuration hooks. The validation procedure
builds the kuavo_assets library target explicitly, then the audited old/new
dependency closure. It does not change those hooks or install dependencies.

CTest includes model alignment, QP regression, freshness, bounded stopping,
reference continuity, RNEA and owner lifecycle tests. Python regressions cover
Mapper transactions, held endpoints, input gating and scheduling. Offline ROS
smoke tests use isolated loopback ROS and synthetic feedback. Build, unit tests
and offline smoke results do not qualify real actuators or a physical closed loop.

Runtime CPU partitioning is in config/runtime_partition.json and must match
the host's online/isolated topology. Offline tests may set
KUAVO_RUNTIME_PARTITION=0; deployment must validate its own topology.
