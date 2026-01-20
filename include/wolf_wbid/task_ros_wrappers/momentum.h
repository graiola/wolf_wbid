// ============================================================================
// File: include/wolf_wbid/task_ros_wrappers/momentum.h
// Updated ROS wrapper (OpenSoT-free)
// ============================================================================

/**
WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
*/

#ifndef TASK_ROS_WRAPPERS_MOMENTUM_H
#define TASK_ROS_WRAPPERS_MOMENTUM_H

// ROS msg (you used CartesianTask msg for this wrapper)
#include <wolf_msgs/CartesianTask.h>

// WoLF
#include <wolf_wbid/task_ros_wrappers/handler.h>
#include <wolf_wbid/task_interface.h>

// OpenSoT-free task
#include <wolf_wbid/wbid/tasks/angular_momentum_task.h>

// Robot & vars
#include <wolf_wbid/quadruped_robot.h>
#include <wolf_wbid/wbid/id_variables.h>

// Utils
#include <wolf_controller_utils/converters.h>

namespace wolf_wbid {

class AngularMomentumImpl : public AngularMomentum, public TaskRosHandler<wolf_msgs::CartesianTask>
{
public:
  using Ptr = std::shared_ptr<AngularMomentumImpl>;

  AngularMomentumImpl(const std::string& robot_name,
                      QuadrupedRobot& robot,
                      const IDVariables& vars,
                      const std::string& task_id = "angular_momentum",
                      const double& period = 0.001);

  void registerReconfigurableVariables() override;
  void loadParams() override;
  void updateCost(const Eigen::VectorXd& x) override;

  void update(const Eigen::VectorXd& x);   // apply buffers + task update
  void publish() override;
  bool reset() override;
};

} // namespace wolf_wbid

#endif // TASK_ROS_WRAPPERS_MOMENTUM_H
