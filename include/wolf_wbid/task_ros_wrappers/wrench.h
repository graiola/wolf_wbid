/**
WoLF: WoLF: Whole-body Lowrenchotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Wrenchmons Attribution-NonWrenchmercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativewrenchmons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef TASK_ROS_WRAPPERS_WRENCH_H
#define TASK_ROS_WRAPPERS_WRENCH_H

// WoLF msgs
#include <wolf_msgs/WrenchTask.h>
#include <wolf_msgs/Wrench.h>

// WoLF
#include <wolf_wbid/task_ros_wrappers/handler.h>

// WoLF utils
#include <wolf_controller_utils/converters.h>

namespace wolf_wbid {

// Wrench
class WrenchImpl : public Wrench, public TaskRosHandler<wolf_msgs::WrenchTask>
{

public:

  typedef std::shared_ptr<WrenchImpl> Ptr;

  WrenchImpl(const std::string& robot_name,
             const std::string& task_id,
             const std::string& distal_link,
             const std::string& base_link,
             OpenSoT::AffineHelper& wrench,
             const double& period);

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void publish() override;

  virtual bool reset() override;

private:

  virtual void _update(const Eigen::VectorXd& x) override;

  void referenceCallback(const wolf_msgs::Wrench::ConstPtr& msg);

  realtime_tools::RealtimeBuffer<Eigen::Vector6d> buffer_reference_;

};

} // namespace

#endif // task_ROS_WRAPPERS_WRENCH_H
