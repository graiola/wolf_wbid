/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef TASK_ROS2_WRAPPERS_POSTURAL_H
#define TASK_ROS2_WRAPPERS_POSTURAL_H

// ROS
#include <wolf_msgs/msg/postural_task.hpp>
#include <wolf_msgs/msg/postural.hpp>
#include <realtime_tools/realtime_buffer.h>

// WoLF
#include <wolf_wbid/ros2/handler.h>
#include <wolf_wbid/core/named_joint_reference.h>

namespace wolf_wbid {

class QuadrupedRobot;
class IDVariables;

// POSTURAL
class PosturalImpl : public Postural, public TaskRosHandler<wolf_msgs::msg::PosturalTask>
{

public:

  typedef std::shared_ptr<PosturalImpl> Ptr;

  PosturalImpl(const std::string& robot_name,
               const std::string& task_id,
               QuadrupedRobot& robot,
               const IDVariables& vars,
               const double& period = 0.001);

  ~PosturalImpl() override { stopSpinner(); }

  virtual void registerReconfigurableVariables() override;

  void activateRuntimeInterfaces() override { TaskRosHandler<wolf_msgs::msg::PosturalTask>::startSpinner(); }

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void publish();

  virtual bool reset() override;

protected:
  void applyExternalKnobs() override;
  void applyExternalReference() override;

private:
  void referenceCallback(const wolf_msgs::msg::Postural::SharedPtr msg);
  rclcpp::Subscription<wolf_msgs::msg::Postural>::SharedPtr reference_sub_;
  realtime_tools::RealtimeBuffer<NamedJointReference> reference_buffer_;
  Eigen::VectorXd reference_positions_;
  Eigen::VectorXd reference_velocities_;
  std::vector<std::size_t> reference_joint_indices_;
  std::vector<std::size_t> leg_reference_indices_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

};

} // namespace

#endif // TASK_ROS2_WRAPPERS_POSTURAL_H
