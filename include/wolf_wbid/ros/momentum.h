/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

/**
WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
*/

#ifndef TASK_ROS_WRAPPERS_MOMENTUM_H
#define TASK_ROS_WRAPPERS_MOMENTUM_H

// ROS message type currently reused from CartesianTask.
#include <wolf_msgs/CartesianTask.h>

// WoLF
#include <wolf_wbid/core/task_interface.h>
#include <wolf_wbid/ros/handler.h>

// STD
#include <memory>
#include <string>

namespace wolf_wbid {

class QuadrupedRobot;
class IDVariables;

/**
 * @brief ROS wrapper for the angular momentum task.
 */
class AngularMomentumImpl : public AngularMomentum, public TaskRosHandler<wolf_msgs::CartesianTask>
{
public:
  using Ptr = std::shared_ptr<AngularMomentumImpl>;

  AngularMomentumImpl(const std::string& robot_name,
                      const std::string& task_id,
                      QuadrupedRobot& robot,
                      const IDVariables& vars,
                      const double& period = 0.001);

  void registerReconfigurableVariables() override;
  void loadParams() override;
  void updateCost(const Eigen::VectorXd& x) override;

  void publish() override;
  bool reset() override;

protected:
  // Called by TaskWrapperInterface::update().
  void applyExternalKnobs() override;
  void applyExternalReference() override;  // Usually unused for momentum.

};

} // namespace wolf_wbid

#endif // TASK_ROS_WRAPPERS_MOMENTUM_H
