#include <wolf_wbid/wbid/tasks/cartesian_task.h>
#include <wolf_wbid/wbid/id_variables.h>
#include <wolf_wbid/quadruped_robot.h>

#include <cmath>
#include <stdexcept>

namespace wolf_wbid {

CartesianTask::CartesianTask(const std::string& task_id,
                             QuadrupedRobot& robot,
                             const std::string& distal_link,
                             const std::string& base_link,
                             const IDVariables& vars)
: task_id_(task_id)
, robot_(robot)
, vars_(vars)
, base_link_(base_link)
, distal_link_(distal_link)
{
  const int n = robot_.getJointNum();
  Bi_.resize(n, n);
  tmp6xn_.resize(6, n);

  // A is 6 x dim(x)
  A_.setZero(6, vars_.size());
  b_.setZero(6);

  // reset ref to current
  resetReference();
}

void CartesianTask::setLambda(double lambda)
{
  if(lambda < 0.0) return;
  lambda1_ = lambda;
  lambda2_ = 2.0 * std::sqrt(lambda1_);
}

void CartesianTask::setLambda(double lambda1, double lambda2)
{
  if(lambda1 < 0.0 || lambda2 < 0.0) return;
  lambda1_ = lambda1;
  lambda2_ = lambda2;
}

void CartesianTask::setOrientationGain(double g)
{
  if(g < 0.0) return;
  orientation_gain_ = g;
}

bool CartesianTask::setBaseLink(const std::string& base_link)
{
  if(base_link == base_link_) return true;

  // We mimic OpenSoT logic: change reference so that it's expressed in new base.
  // We rely on robot_.getPose(q, source, target, T) OR runtime pose getters.
  // Here we assume robot_ is already updated and can provide link poses.

  Eigen::Affine3d T_newBase_oldBase = Eigen::Affine3d::Identity();

  if(base_link == "world")
  {
    // world_T_oldBase
    Eigen::Affine3d world_T_oldBase;
    if(!robot_.getPose(robot_.getJointPosition(), base_link_, "world", T_newBase_oldBase)) {
      // fallback if you have a direct getter:
      // robot_.getPose(base_link_, world_T_oldBase);
    }
  }

  // More robust: use helper getPose(q, source_frame, target_frame, pose)
  Eigen::Affine3d oldBase_T_newBase = Eigen::Affine3d::Identity();
  if(!robot_.getPose(robot_.getJointPosition(), base_link, base_link_, oldBase_T_newBase))
    return false;

  // old pose_ref is in oldBase; new pose_ref should be in newBase:
  // newBase_T_distal_ref = (newBase_T_oldBase) * (oldBase_T_distal_ref)
  Eigen::Affine3d newBase_T_oldBase = oldBase_T_newBase.inverse();
  pose_ref_ = newBase_T_oldBase * pose_ref_;

  base_link_ = base_link;
  return true;
}

bool CartesianTask::setDistalLink(const std::string& distal_link)
{
  if(distal_link == distal_link_) return true;
  if(distal_link == "world") return false;

  // Set new reference to current pose of the new distal link (in current base)
  distal_link_ = distal_link;
  resetReference();
  return true;
}

void CartesianTask::setReference(const Eigen::Affine3d& pose_ref)
{
  pose_ref_ = pose_ref;
  vel_ref_.setZero();
  acc_ref_.setZero();
  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
}

void CartesianTask::setReference(const Eigen::Affine3d& pose_ref, const Eigen::Matrix<double,6,1>& vel_ref)
{
  pose_ref_ = pose_ref;
  vel_ref_  = vel_ref;
  acc_ref_.setZero();
  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
}

void CartesianTask::setReference(const Eigen::Affine3d& pose_ref,
                                 const Eigen::Matrix<double,6,1>& vel_ref,
                                 const Eigen::Matrix<double,6,1>& acc_ref)
{
  pose_ref_ = pose_ref;
  vel_ref_  = vel_ref;
  acc_ref_  = acc_ref;
  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
}

void CartesianTask::setVirtualForce(const Eigen::Matrix<double,6,1>& virtual_force_ref)
{
  virtual_force_ref_ = virtual_force_ref;
}

void CartesianTask::getReference(Eigen::Affine3d& pose_ref) const
{
  pose_ref = pose_ref_;
}

void CartesianTask::getReference(Eigen::Affine3d& pose_ref, Eigen::Matrix<double,6,1>& vel_ref) const
{
  pose_ref = pose_ref_;
  vel_ref  = vel_ref_;
}

void CartesianTask::resetReference()
{
  // we use current state (robot_ already has it), compute pose in base_link_
  Eigen::Affine3d pose;
  const auto q = robot_.getJointPosition(); // if available; otherwise store q externally
  if(!robot_.getPose(q, distal_link_, base_link_, pose))
  {
    // if base is world and you have getPose(link, pose):
    if(!robot_.getPose(q, distal_link_, pose))
      pose = Eigen::Affine3d::Identity();
  }
  pose_ref_ = pose;
}

bool CartesianTask::reset()
{
  resetReference();
  vel_ref_.setZero();
  acc_ref_.setZero();
  virtual_force_ref_.setZero();
  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
  virtual_force_ref_cached_ = virtual_force_ref_;
  return true;
}

// simple SO(3) orientation error (log map approx using skew part)
// (OpenSoT uses XBot::Utils::computeOrientationError; we replicate a standard version)
void CartesianTask::computeOrientationError(const Eigen::Matrix3d& R_des,
                                            const Eigen::Matrix3d& R_act,
                                            Eigen::Vector3d& e) const
{
  const Eigen::Matrix3d R_err = R_des * R_act.transpose();
  Eigen::Vector3d v;
  v << (R_err(2,1) - R_err(1,2)),
       (R_err(0,2) - R_err(2,0)),
       (R_err(1,0) - R_err(0,1));
  e = 0.5 * v; // small-angle approx
}

void CartesianTask::computeCartesianInertiaInverse()
{
  // Mi = (J * B^{-1} * J^T)
  robot_.getInertiaInverse(Bi_); // Bi_ = B^{-1}
  tmp6xn_.noalias() = J_ * Bi_;
  Mi_.noalias()     = tmp6xn_ * J_.transpose();
}

void CartesianTask::update(const Eigen::VectorXd& x)
{
  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
  virtual_force_ref_cached_ = virtual_force_ref_;

  const auto qddot = vars_.qddot(x);  // Map, no copy

  // get robot state once
  const Eigen::VectorXd& q  = robot_.getJointPosition();
  const Eigen::VectorXd& qd = robot_.getJointVelocity();

  // --- Kinematics in base_link frame (preferred; no silent world fallback) ---
  if(!robot_.getJacobian(q, distal_link_, base_link_, J_))
    throw std::runtime_error("CartesianTask::update(): missing relative Jacobian API");

  if(!robot_.getPose(q, distal_link_, base_link_, pose_current_))
    throw std::runtime_error("CartesianTask::update(): missing relative pose API");

  if(!robot_.getTwist(q, qd, distal_link_, base_link_, vel_current_))
    vel_current_.setZero(); // ok as temporary fallback, but mark TODO

  jdotqdot_.setZero(); // TODO when available

  computeOrientationError(pose_ref_.linear(), pose_current_.linear(), orientation_error_);

  pose_error_.head<3>() = pose_ref_.translation() - pose_current_.translation();
  pose_error_.tail<3>() = orientation_gain_ * orientation_error_;

  vel_error_ = vel_ref_ - vel_current_;

  const Eigen::Matrix<double,6,1> cart_acc = J_ * qddot + jdotqdot_;

  Eigen::Matrix<double,6,1> y;
  if(gain_type_ == GainType::Acceleration)
  {
    y = acc_ref_
        + lambda2_ * (Kd_ * vel_error_)
        + lambda1_ * (Kp_ * pose_error_);
  }
  else
  {
    computeCartesianInertiaInverse();
    y = acc_ref_
        + lambda2_ * (Mi_ * (Kd_ * vel_error_))
        + lambda1_ * (Mi_ * (Kp_ * pose_error_))
        + Mi_ * virtual_force_ref_;
  }

  // A picks qddot block
  A_.setZero();
  const auto& qb = vars_.qddotBlock();
  A_.block(0, qb.offset, 6, qb.dim) = J_;
  b_ = y - jdotqdot_;

  vel_ref_.setZero();
  acc_ref_.setZero();
  virtual_force_ref_.setZero();
}


} // namespace wolf_wbid
