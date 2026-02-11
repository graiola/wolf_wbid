/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

/**
WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
*/

#ifndef TASK_ROS_WRAPPERS_WRENCH_H
#define TASK_ROS_WRAPPERS_WRENCH_H

// ROS msgs
#include <wolf_msgs/WrenchTask.h>
#include <wolf_msgs/Wrench.h>

// WoLF
#include <wolf_wbid/core/task_interface.h>
#include <wolf_wbid/ros/handler.h>

// ROS RT buffer
#include <realtime_tools/realtime_buffer.h>

// Eigen
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace wolf_wbid {

class IDVariables;

/**
 * ROS wrapper for OpenSoT-free POINT_CONTACT wrench task
 * Minimizes || f_contact - f_ref || with scalar weight.
 */
class WrenchImpl : public Wrench, public TaskRosHandler<wolf_msgs::WrenchTask>
{
public:
  using Ptr = std::shared_ptr<WrenchImpl>;

  WrenchImpl(const std::string& robot_name,
             const std::string& task_id,
             const std::string& contact_name,
             const IDVariables& vars,
             const double& period = 0.001);

  void registerReconfigurableVariables() override;
  void loadParams() override;
  void updateCost(const Eigen::VectorXd& x) override;

  void publish() override;
  bool reset() override;

protected:
  // called by TaskWrapperInterface::update(x)
  void applyExternalKnobs() override;
  void applyExternalReference() override;

private:
  void referenceCallback(const wolf_msgs::Wrench::ConstPtr& msg);

  const IDVariables& vars_;

  realtime_tools::RealtimeBuffer<Eigen::Vector3d> buffer_reference_force_;

  // Optional: cache last x if you want to publish actual wrench from solution
  // (set from updateCost or from solver externally)
  Eigen::Vector3d last_f_act_{Eigen::Vector3d::Zero()};
  bool has_last_f_act_{false};
};

} // namespace wolf_wbid

#endif // TASK_ROS_WRAPPERS_WRENCH_H
