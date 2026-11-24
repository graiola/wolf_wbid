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
activation, zero-force bounds and the contact contributions to dynamics and
torque constraints. The update applies the planned mode before rebuilding the
constraints. WPG remains governed by the gait generator. EXT clients that never
provide a planned contact schedule keep their previous all-contact behavior.

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
