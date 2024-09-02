/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef TASK_ROS2_WRAPPERS_POSTURAL_H
#define TASK_ROS2_WRAPPERS_POSTURAL_H

// ROS
#include <wolf_msgs/msg/postural_task.hpp>

// WoLF
#include <wolf_wbid/task_ros2_wrappers/handler.h>

namespace wolf_wbid {

// POSTURAL
class PosturalImpl : public Postural, public TaskRosHandler<wolf_msgs::msg::PosturalTask>
{

public:

  typedef std::shared_ptr<PosturalImpl> Ptr;

  PosturalImpl(const std::string& robot_name,
               const XBot::ModelInterface& robot,
               OpenSoT::AffineHelper qddot = OpenSoT::AffineHelper(),
               const std::string& task_id = "postural",
               const double& period = 0.001);

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void publish();

  virtual bool reset() override;

private:

  virtual void _update(const Eigen::VectorXd& x) override;

};

} // namespace

#endif // TASK_ROS2_WRAPPERS_POSTURAL_H

