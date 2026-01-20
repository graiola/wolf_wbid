/**
WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
*/

#ifndef TASK_ROS_WRAPPERS_WRENCH_H
#define TASK_ROS_WRAPPERS_WRENCH_H

// ROS msgs
#include <wolf_msgs/WrenchTask.h>
#include <wolf_msgs/Wrench.h>

// WoLF
#include <wolf_wbid/task_ros_wrappers/handler.h>
#include <wolf_wbid/task_interface.h>

// New OpenSoT-free task
#include <wolf_wbid/wbid/tasks/wrench_task.h>
#include <wolf_wbid/wbid/id_variables.h>

// ROS RT buffer
#include <realtime_tools/realtime_buffer.h>

// Eigen
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace wolf_wbid {

/**
 * ROS wrapper for OpenSoT-free POINT_CONTACT wrench task
 * Minimizes || f_contact - f_ref || with diagonal scalar weight.
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

  // called by IDProblem/update loop (like others)
  void update(const Eigen::VectorXd& x);

private:
  void referenceCallback(const wolf_msgs::Wrench::ConstPtr& msg);

  const IDVariables& vars_;

  realtime_tools::RealtimeBuffer<Eigen::Vector3d> buffer_reference_force_;
};

} // namespace wolf_wbid

#endif // TASK_ROS_WRAPPERS_WRENCH_H
