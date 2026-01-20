/**
 * WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
 */

#ifndef TASK_ROS_WRAPPERS_POSTURAL_H
#define TASK_ROS_WRAPPERS_POSTURAL_H

#include <memory>

// ROS msgs
#include <wolf_msgs/PosturalTask.h>

// WoLF
#include <wolf_wbid/wbid/tasks/postural_task.h>
#include <wolf_wbid/task_ros_wrappers/handler.h>
#include <wolf_wbid/task_interface.h>   // la versione "micro" solo wrapper (senza OpenSoT)

namespace wolf_wbid {

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
  void _update(const Eigen::VectorXd& x) override;

private:
  double cost_{0.0};
};

} // namespace wolf_wbid

#endif // TASK_ROS_WRAPPERS_POSTURAL_H
