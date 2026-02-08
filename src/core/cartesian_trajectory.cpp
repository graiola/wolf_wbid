/**
 * @file cartesian_trajectory.cpp
 * @author Gennaro Raiola
 * @date 1 December, 2021
 * @brief Cartesian trajectory (OpenSoT-free)
 */

#include <wolf_wbid/core/cartesian_trajectory.h>

// New OpenSoT-free task
#include <wolf_wbid/wbid/tasks/cartesian_task.h>

using namespace wolf_controller_utils;
using namespace trajectory;

namespace
{
constexpr double DEFAULT_REACH_THRESHOLD = 1e-6;
}

namespace wolf_wbid
{

CartesianTrajectory::CartesianTrajectory(wolf_wbid::CartesianTask* task_ptr)
  : state_(state_t::IDLE),
    trajectory_(std::make_shared<Trajectory>()),
    task_ptr_(task_ptr)
{
  reset();
}

void CartesianTrajectory::setTask(wolf_wbid::CartesianTask* task_ptr)
{
  task_ptr_ = task_ptr;
}

void CartesianTrajectory::reset()
{
  state_ = state_t::IDLE;

  if(task_ptr_ != nullptr)
  {
    // CartesianTask API: getActualPose(Eigen::Affine3d&)
    task_ptr_->getActualPose(T_);
  }
  else
  {
    T_.setIdentity();
  }

  vel_.setZero();
  acc_.setZero();

  if(trajectory_) trajectory_->clear();
  time_ = 0.0;
}

void CartesianTrajectory::update(double period)
{
  switch(state_.load())
  {
    case state_t::EXECUTING:
    {
      T_ = trajectory_->evaluate(time_.load(), &vel_, &acc_);

      const double t = time_.load();
      if(trajectory_->isTrajectoryEnded(t) && checkProximity())
      {
        state_ = state_t::IDLE;
      }

      time_ = t + period;
      break;
    }

    case state_t::IDLE:
    default:
      vel_.setZero();
      acc_.setZero();
      time_ = 0.0;
      break;
  }
}

bool CartesianTrajectory::getReference(Eigen::Affine3d& T_ref,
                                       Eigen::Matrix<double,6,1>* vel_ref,
                                       Eigen::Matrix<double,6,1>* acc_ref) const
{
  T_ref = T_;
  if(vel_ref) *vel_ref = vel_;
  if(acc_ref) *acc_ref = acc_;
  return true;
}

bool CartesianTrajectory::setWayPoint(const Eigen::Affine3d& T_ref, double duration)
{
  if(state_.load() == state_t::EXECUTING)
    return false;

  state_ = state_t::EXECUTING;

  trajectory_->clear();
  trajectory_->addWayPoint(0.0, T_);
  trajectory_->addWayPoint(duration, T_ref);
  trajectory_->compute();

  return true;
}

bool CartesianTrajectory::setWayPoints(const WayPointVector& wps)
{
  if(state_.load() == state_t::EXECUTING)
    return false;

  state_ = state_t::EXECUTING;

  trajectory_->clear();
  trajectory_->addWayPoint(0.0, T_);

  for(unsigned int i = 0; i < wps.size(); i++)
  {
    trajectory_->addWayPoint(wps[i].duration, wps[i].T_ref);
  }

  trajectory_->compute();
  return true;
}

CartesianTrajectory::state_t CartesianTrajectory::getState() const
{
  return state_.load();
}

bool CartesianTrajectory::checkProximity() const
{
  if(!trajectory_ || trajectory_->getWayPoints().empty())
    return true;

  const auto T_ref = trajectory_->getWayPoints().back().frame_;
  return T_ref.isApprox(T_, DEFAULT_REACH_THRESHOLD);
}

double CartesianTrajectory::getTime() const
{
  return time_.load();
}

} // namespace wolf_wbid
