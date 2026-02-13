/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef TASK_ROS2_WRAPPERS_WRENCH_H
#define TASK_ROS2_WRAPPERS_WRENCH_H

// WoLF msgs
#include <wolf_msgs/msg/wrench_task.hpp>
#include <wolf_msgs/msg/wrench.hpp>

// WoLF
#include <wolf_wbid/ros2/handler.h>

// WoLF utils
#include <wolf_controller_utils/converters.h>

namespace wolf_wbid {

class IDVariables;

// Wrench
class WrenchImpl : public Wrench, public TaskRosHandler<wolf_msgs::msg::WrenchTask>
{

public:

  typedef std::shared_ptr<WrenchImpl> Ptr;

  WrenchImpl(const std::string& robot_name,
             const std::string& task_id,
             const std::string& contact_name,
             const IDVariables& vars,
             const double& period = 0.001);

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void publish() override;

  virtual bool reset() override;

protected:
  void applyExternalKnobs() override;
  void applyExternalReference() override;

private:

  void referenceCallback(const wolf_msgs::msg::Wrench::SharedPtr msg);

  const IDVariables& vars_;
  realtime_tools::RealtimeBuffer<Eigen::Vector3d> buffer_reference_force_;
  Eigen::Vector3d last_f_act_{Eigen::Vector3d::Zero()};
  bool has_last_f_act_{false};

  typename rclcpp::Subscription<wolf_msgs::msg::Wrench>::SharedPtr reference_sub_;

};

} // namespace

#endif // TASK_ROS2_WRAPPERS_WRENCH_H
