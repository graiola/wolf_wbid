/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef WOLF_WBID_CARTESIAN_TRAJECTORY_H
#define WOLF_WBID_CARTESIAN_TRAJECTORY_H

#include <atomic>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <wolf_controller_utils/trajectory/trajectory.h>

// OpenSoT-free task (forward decl is enough here)
namespace wolf_wbid {
class CartesianTask;
}

namespace wolf_wbid
{

class CartesianTrajectory
{
public:
  enum state_t { IDLE = 0, EXECUTING };

  struct WayPoint
  {
    Eigen::Affine3d T_ref{Eigen::Affine3d::Identity()};
    double duration{0.0};
  };

  using WayPointVector = std::vector<WayPoint>;

  const std::string CLASS_NAME = "CartesianTrajectory";

  using Ptr = std::shared_ptr<CartesianTrajectory>;
  using ConstPtr = std::shared_ptr<const CartesianTrajectory>;

  explicit CartesianTrajectory(wolf_wbid::CartesianTask* task_ptr = nullptr);

  void setTask(wolf_wbid::CartesianTask* task_ptr);

  void update(double period);
  void reset();

  double getTime() const;

  bool getReference(Eigen::Affine3d& T_ref,
                    Eigen::Matrix<double,6,1>* vel_ref = nullptr,
                    Eigen::Matrix<double,6,1>* acc_ref = nullptr) const;

  bool setWayPoint(const Eigen::Affine3d& T_ref, double duration);
  bool setWayPoints(const WayPointVector& wps);

  state_t getState() const;

private:
  bool checkProximity() const;

  std::atomic<double> time_{0.0};
  std::atomic<state_t> state_{state_t::IDLE};

  Eigen::Affine3d T_{Eigen::Affine3d::Identity()};
  Eigen::Matrix<double,6,1> vel_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> acc_{Eigen::Matrix<double,6,1>::Zero()};

  wolf_controller_utils::trajectory::Trajectory::Ptr trajectory_;
  wolf_wbid::CartesianTask* task_ptr_{nullptr};
};

} // namespace wolf_wbid

#endif // WOLF_WBID_CARTESIAN_TRAJECTORY_H
