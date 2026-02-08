/**
 * WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
 */

#ifndef TASK_ROS_WRAPPERS_POSTURAL_H
#define TASK_ROS_WRAPPERS_POSTURAL_H

#include <memory>
#include <string>

// ROS msg
#include <wolf_msgs/PosturalTask.h>

// WoLF
#include <wolf_wbid/core/task_interface.h>
#include <wolf_wbid/ros/handler.h>

namespace wolf_wbid {

class QuadrupedRobot;
class IDVariables;

class PosturalImpl : public Postural, public TaskRosHandler<wolf_msgs::PosturalTask>
{
public:
  using Ptr = std::shared_ptr<PosturalImpl>;

  PosturalImpl(const std::string& robot_name,
               const std::string& task_id,
               QuadrupedRobot& robot,
               const IDVariables& idvars,
               double period = 0.001);

  void registerReconfigurableVariables() override;
  void loadParams() override;
  void updateCost(const Eigen::VectorXd& x) override;

  void publish() override;
  bool reset() override;

protected:
  // called by TaskWrapperInterface::update(x)
  void applyExternalKnobs() override;
  void applyExternalReference() override;  // currently unused for postural (unless you add a topic later)
};

} // namespace wolf_wbid

#endif // TASK_ROS_WRAPPERS_POSTURAL_H
