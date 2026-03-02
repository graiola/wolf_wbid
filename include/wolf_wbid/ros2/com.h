/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef TASK_ROS2_WRAPPERS_COM_H
#define TASK_ROS2_WRAPPERS_COM_H

// WoLF msgs
#include <wolf_msgs/msg/com_task.hpp>
#include <wolf_msgs/msg/com.hpp>

// WoLF
#include <wolf_wbid/ros2/handler.h>

// WoLF utils
#include <wolf_controller_utils/converters.h>

namespace wolf_wbid {

class QuadrupedRobot;
class IDVariables;

// Com
class ComImpl : public Com, public TaskRosHandler<wolf_msgs::msg::ComTask>
{

public:

  typedef std::shared_ptr<ComImpl> Ptr;

  ComImpl(const std::string& robot_name,
          const std::string& task_id,
          QuadrupedRobot& robot,
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

  void referenceCallback(const wolf_msgs::msg::Com::SharedPtr msg);

  realtime_tools::RealtimeBuffer<Eigen::Vector3d> buffer_reference_pos_;
  realtime_tools::RealtimeBuffer<Eigen::Vector3d> buffer_reference_vel_;

  typename rclcpp::Subscription<wolf_msgs::msg::Com>::SharedPtr reference_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

};

} // namespace

#endif // TASK_ROS2_WRAPPERS_COM_H
