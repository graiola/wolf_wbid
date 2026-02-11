# `wolf_wbid`

Whole-body inverse dynamics package in WoLF for quadruped control.

It provides:
- inverse-dynamics problem setup (`id_problem`)
- task/constraint building blocks (Cartesian, CoM, momentum, postural, wrench)
- ROS1/ROS2 wrappers
- QP solvers (`qpOASES` default, optional `eiQuadProg`)

## Dependencies

`tf2`, `tf2_ros`, `tf2_eigen`, `realtime_tools`, `interactive_markers`,
`kdl_parser`, `wolf_msgs`, `wolf_controller_utils`.

## Build

ROS1:

```bash
catkin build wolf_wbid
source devel/setup.bash
```

ROS2:

```bash
colcon build --packages-select wolf_wbid
source install/setup.bash
```

Optional `eiQuadProg`:

```bash
catkin config --cmake-args -DWOLF_WBID_ENABLE_EIQUADPROG=ON
# or:
colcon build --cmake-args -DWOLF_WBID_ENABLE_EIQUADPROG=ON
```
