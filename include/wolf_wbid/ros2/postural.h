/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef TASK_ROS2_WRAPPERS_POSTURAL_H
#define TASK_ROS2_WRAPPERS_POSTURAL_H

// ROS
#include <wolf_msgs/msg/postural_task.hpp>

// WoLF
#include <wolf_wbid/ros2/handler.h>

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

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void publish();

  virtual bool reset() override;

protected:
  void applyExternalKnobs() override;
  void applyExternalReference() override;

};

} // namespace

#endif // TASK_ROS2_WRAPPERS_POSTURAL_H
