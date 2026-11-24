# External MPC references in ROS 2

The task wrappers accept references beneath `/<robot>/wolf_controller/reference/`
with reliable depth-one QoS. MPC needs four foot Cartesian messages, four point
forces, one waist Cartesian message and one named postural message. The EXT task
stack uses full waist pose and excludes CoM, so no CoM reference is required.

## Named postural targets

`PosturalImpl` subscribes to `postural` using `wolf_msgs/msg/Postural`. Names are
mandatory and identify actuated joints. Positions must match the name count;
velocities must either match or be empty (zero desired velocity). Unknown names,
duplicates, mismatched arrays and non-finite values reject the whole command.
Named subsets are supported outside MPC. MPC requires all leg joints in each
postural message, while omitted arm joints keep their existing targets. An arm
or partial-leg update cannot renew MPC postural readiness.

Real model indices map names to the full task vector, including the floating-base
offset. A single realtime buffer keeps positions and velocities together.
Reset invalidates earlier commands with a generation counter, including a
callback that began before reset and finished afterward. Once a planned MPC
stream has been adopted, the buffered references survive the WPG-to-EXT reset.

The `wolf_controller/postural` telemetry topic publishes
`wolf_msgs/msg/PosturalTask` with exactly one entry per named actuated joint,
including arm joints when present. All six position, velocity, reference and
error arrays use the same name-to-model index mapping as the reference receiver.
Floating-base rows are excluded. Previously, this message paired actuated names
with the first generalized-coordinate rows and appended six empty names, so its
named reference values did not describe the task values applied by WBID.
For a quadruped with twelve leg joints, these arrays now contain twelve entries;
arm joints add their own named entries. Consumers that assumed eighteen
generalized-coordinate entries must read the actuated values by `message.name`
instead. This corrects the published data layout without changing the ROS
message definition.

## Cartesian and force references

A Cartesian message contains finite position and twist values and a finite,
nonzero quaternion; valid quaternions are normalized. Its pose, twist and frame
share one buffered snapshot. Frame changes happen on the controller thread,
not in the ROS callback. MPC requires explicit `world`; unsupported model frames
are rejected. Interactive marker edits do not replace Cartesian targets while
the adopted MPC stream owns them.

Point-force messages require finite force and torque values. Forces act in world
axes; there is no implicit TF conversion. Explicit non-world frames are rejected.
An empty frame remains accepted for generic legacy point-force use, but cannot
renew MPC readiness. Message stamps remain useful for diagnostics; steady-clock
receipt time governs timeout checks.

## Contact constraints and readiness

`IDProblem::setPlannedContactStates()` is a controller-thread API. The four flags
follow `QuadrupedRobot::getFootNames()` order. In EXT they control friction-cone
activation, zero-force bounds and the contact contributions to floating-base
dynamics. The update applies the planned mode before rebuilding the
constraints. WPG remains governed by the gait generator. EXT clients that never
provide a planned contact schedule keep their previous all-contact behavior.

Released contacts have exact zero force bounds in the assembled EXT QP. Their
force variables must be zero because the floating-base dynamics exclude those
contacts while torque reconstruction reads the full force vector. These bound
objects participate in both constraint stacks, with different activation:
WPG retains its configured stance minimum and force limits; EXT activates only
the zero bounds for released contacts. EXT stance forces retain their previous
friction-cone constraints without adopting WPG's positive force minimum or
finite force caps. This also preserves legacy EXT with all contacts enabled.

`externalMpcReferencesReady()` checks the accepted receipt generation and age of
each of the ten channels. A contact heartbeat cannot conceal a missing or stale
foot, force, waist or postural reference. These independent checks enforce
bounded age, not an atomic policy snapshot across separate DDS topics.

The ROS contact transport, guarded `MPC` service action and stale-reference
emergency response live in `wolf_controller`; this library does not own a second
watchdog. Do not call the planned-contact setters directly from a ROS subscription
callback. The controller validates named commands off the control thread, then
applies a buffered snapshot before its update.

## Validation

`test/named_joint_reference_test.cpp` checks name mapping, complete leg coverage,
partial leg references preserving arm targets, zero default velocities,
non-finite values and malformed messages. `test/reference_readiness_test.cpp`
checks missing and stale individual channels, reset/callback generation races,
Cartesian quaternion validity and point-force frame validation. Both are
standalone C++17 tests; the second also needs Eigen. Full task execution and
contact transitions require the coordinated ROS 2 build and simulation
validation described in `wolf_planner`'s migration plan.

In ROS 2 builds, `test/postural_telemetry_test.cpp` checks the actual message
assembly used by `PosturalImpl::publish()`: shuffled generalized indices, six
floating-base rows, an arm joint, all six aligned arrays and the existing
zero-velocity-reference fallback. It uses the generated ROS 2 message type and
Eigen without starting ROS nodes. The private assembly helper is not installed
and changes no public C++ API or class layout.

On 2026-11-24, the native ROS 2 Humble build with Eigen 3.4.1 configured, built
and installed successfully, and all three CTest entries passed: named joint
references, reference readiness and postural telemetry. These checks establish
message assembly and local reference contracts. Simulator and hardware
execution are **NOT_RUN** by these tests; the coordinated runtime results are
recorded in the planner migration validation ledger.
