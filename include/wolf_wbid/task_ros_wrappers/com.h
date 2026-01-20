#pragma once

#include <wolf_wbid/task_interface.h>
#include <wolf_wbid/task_ros_wrappers/handler.h>

#include <wolf_msgs/ComTask.h>
#include <wolf_msgs/Com.h>

#include <realtime_tools/realtime_buffer.h>

namespace wolf_wbid {

class ComImpl : public Com, public TaskRosHandler<wolf_msgs::ComTask>
{
public:
  using Ptr = std::shared_ptr<ComImpl>;

  ComImpl(const std::string& robot_name,
          const std::string& task_id,
          QuadrupedRobot& robot,
          const IDVariables& vars,
          const double& period = 0.001);

  void registerReconfigurableVariables() override;
  void loadParams() override;
  void updateCost(const Eigen::VectorXd& x) override;
  void publish() override;
  bool reset() override;

private:

  void _update(const Eigen::VectorXd& x) override;  
  void referenceCallback(const wolf_msgs::Com::ConstPtr& msg);

  realtime_tools::RealtimeBuffer<Eigen::Vector3d> buffer_reference_pos_;
  realtime_tools::RealtimeBuffer<Eigen::Vector3d> buffer_reference_vel_;
};

} // namespace wolf_wbid
