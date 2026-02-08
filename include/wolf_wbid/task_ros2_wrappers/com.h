/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef TASK_ROS2_WRAPPERS_COM_H
#define TASK_ROS2_WRAPPERS_COM_H

// WoLF msgs
#include <wolf_msgs/msg/com_task.hpp>
#include <wolf_msgs/msg/com.hpp>

// WoLF
#include <wolf_wbid/task_ros2_wrappers/handler.h>

// WoLF utils
#include <wolf_controller_utils/converters.h>

namespace wolf_wbid {

// Com
class ComImpl : public Com, public TaskRosHandler<wolf_msgs::msg::ComTask>
{

public:

  typedef std::shared_ptr<ComImpl> Ptr;

  ComImpl(const std::string& robot_name,
          QuadrupedRobot& robot,
          const OpenSoT::AffineHelper& qddot,
          const double& period = 0.001);

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void publish() override;

  virtual bool reset() override;

private:

  virtual void _update(const Eigen::VectorXd& x) override;

  void referenceCallback(const wolf_msgs::msg::Com::SharedPtr msg);

  realtime_tools::RealtimeBuffer<Eigen::Vector3d> buffer_reference_pos_;
  realtime_tools::RealtimeBuffer<Eigen::Vector3d> buffer_reference_vel_;

  typename rclcpp::Subscription<wolf_msgs::msg::Com>::SharedPtr reference_sub_;

};

} // namespace

#endif // TASK_ROS2_WRAPPERS_COM_H
