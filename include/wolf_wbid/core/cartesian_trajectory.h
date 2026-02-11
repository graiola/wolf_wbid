/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef WOLF_WBID_CARTESIAN_TRAJECTORY_H
#define WOLF_WBID_CARTESIAN_TRAJECTORY_H

#include <atomic>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <wolf_controller_utils/trajectory/trajectory.h>

// Forward declaration is enough in this header.
namespace wolf_wbid {
class CartesianTask;
}

namespace wolf_wbid
{

/**
 * @brief Time-parameterized Cartesian trajectory helper for a CartesianTask.
 */
class CartesianTrajectory
{
public:
  /** @brief Trajectory execution state. */
  enum state_t { IDLE = 0, EXECUTING };

  /** @brief Cartesian waypoint with pose and segment duration. */
  struct WayPoint
  {
    Eigen::Affine3d T_ref{Eigen::Affine3d::Identity()};
    double duration{0.0};
  };

  using WayPointVector = std::vector<WayPoint>;

  const std::string CLASS_NAME = "CartesianTrajectory";

  using Ptr = std::shared_ptr<CartesianTrajectory>;
  using ConstPtr = std::shared_ptr<const CartesianTrajectory>;

  /**
   * @brief Constructs the trajectory helper.
   * @param task_ptr Optional task pointer used for proximity checks and reset behavior.
   */
  explicit CartesianTrajectory(wolf_wbid::CartesianTask* task_ptr = nullptr);

  /** @brief Sets the task used by this trajectory helper. */
  void setTask(wolf_wbid::CartesianTask* task_ptr);

  /** @brief Advances the internal trajectory clock and state. */
  void update(double period);
  /** @brief Clears waypoints and resets execution state. */
  void reset();

  /** @brief Returns elapsed trajectory time in seconds. */
  double getTime() const;

  bool getReference(Eigen::Affine3d& T_ref,
                    Eigen::Matrix<double,6,1>* vel_ref = nullptr,
                    Eigen::Matrix<double,6,1>* acc_ref = nullptr) const;

  /** @brief Appends a waypoint segment. */
  bool setWayPoint(const Eigen::Affine3d& T_ref, double duration);
  /** @brief Replaces the trajectory with a sequence of waypoints. */
  bool setWayPoints(const WayPointVector& wps);

  /** @brief Returns the current execution state. */
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
