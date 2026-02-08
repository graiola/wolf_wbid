/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef TASK_ROS2_WRAPPERS_MOMENTUM_H
#define TASK_ROS2_WRAPPERS_MOMENTUM_H

// ROS
#include <wolf_msgs/msg/cartesian_task.hpp>

// WoLF
#include <wolf_wbid/task_ros2_wrappers/handler.h>

namespace wolf_wbid {

// AngularMomentum
class AngularMomentumImpl : public AngularMomentum, public TaskRosHandler<wolf_msgs::msg::CartesianTask>
{

public:

  typedef std::shared_ptr<AngularMomentumImpl> Ptr;

  AngularMomentumImpl(const std::string& robot_name,
                      QuadrupedRobot& robot,
                      const OpenSoT::AffineHelper& qddot,
                      const double& period = 0.001);

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void update(const Eigen::VectorXd& x);

  virtual void publish();

  virtual bool reset() override;

};

} // namespace

#endif // TASK_ROS2_WRAPPERS_MOMENTUM_H
