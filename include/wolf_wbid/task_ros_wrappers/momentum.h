/**
WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
*/

#ifndef TASK_ROS_WRAPPERS_MOMENTUM_H
#define TASK_ROS_WRAPPERS_MOMENTUM_H

// ROS msg (legacy: reusing CartesianTask msg)
#include <wolf_msgs/CartesianTask.h>

// WoLF
#include <wolf_wbid/task_interface.h>
#include <wolf_wbid/task_ros_wrappers/handler.h>

// STD
#include <memory>
#include <string>

namespace wolf_wbid {

class QuadrupedRobot;
class IDVariables;

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
  // called by TaskWrapperInterface::update(x)
  void applyExternalKnobs() override;
  void applyExternalReference() override;  // typically unused for momentum

};

} // namespace wolf_wbid

#endif // TASK_ROS_WRAPPERS_MOMENTUM_H
