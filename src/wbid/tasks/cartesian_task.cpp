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

  q_.setZero(n);
  qd_.setZero(n);

  // reset ref to current
  resetReference();
}

bool CartesianTask::readRobotState()
{
  if(!robot_.getJointPosition(q_)) return false;
  if(!robot_.getJointVelocity(qd_)) return false;
  return true;
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

  // Need current state to remap reference as OpenSoT does
  if(!readRobotState()) return false;

  // OpenSoT logic:
  // pose_ref is currently expressed in old base.
  // We want the same physical target expressed in new base:
  // newBase_T_distal_ref = (newBase_T_oldBase) * (oldBase_T_distal_ref)
  Eigen::Affine3d oldBase_T_newBase = Eigen::Affine3d::Identity();
  if(!robot_.getPose(q_, base_link_, base_link, oldBase_T_newBase))
  {
    // if links invalid this will fail
    return false;
  }

  const Eigen::Affine3d newBase_T_oldBase = oldBase_T_newBase.inverse();
  pose_ref_ = newBase_T_oldBase * pose_ref_;

  base_link_ = base_link;
  return true;
}

bool CartesianTask::setDistalLink(const std::string& distal_link)
{
  if(distal_link == distal_link_) return true;
  if(distal_link == "world") return false;

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

void CartesianTask::setReference(const Eigen::Affine3d& pose_ref,
                                 const Eigen::Matrix<double,6,1>& vel_ref)
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

void CartesianTask::getReference(Eigen::Affine3d& pose_ref,
                                 Eigen::Matrix<double,6,1>& vel_ref) const
{
  pose_ref = pose_ref_;
  vel_ref  = vel_ref_;
}

void CartesianTask::resetReference()
{
  if(!readRobotState())
  {
    pose_ref_.setIdentity();
    return;
  }

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  if(!robot_.getPose(q_, distal_link_, base_link_, pose))
  {
    // keep safe default
    pose.setIdentity();
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

// small-angle SO(3) orientation error (OpenSoT uses XBot::Utils::computeOrientationError)
void CartesianTask::computeOrientationError(const Eigen::Matrix3d& R_des,
                                            const Eigen::Matrix3d& R_act,
                                            Eigen::Vector3d& e) const
{
  const Eigen::Matrix3d R_err = R_des * R_act.transpose();
  Eigen::Vector3d v;
  v << (R_err(2,1) - R_err(1,2)),
       (R_err(0,2) - R_err(2,0)),
       (R_err(1,0) - R_err(0,1));
  e = 0.5 * v;
}

void CartesianTask::computeCartesianInertiaInverse()
{
  // Mi = (J * B^{-1} * J^T)
  robot_.getInertiaInverse(Bi_);
  tmp6xn_.noalias() = J_ * Bi_;
  Mi_.noalias()     = tmp6xn_ * J_.transpose();
}

void CartesianTask::update(const Eigen::VectorXd& x)
{
  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
  virtual_force_ref_cached_ = virtual_force_ref_;

  const auto qddot = vars_.qddot(x); // Map

  // Read robot state once (new interfaces: getters require output arg)
  if(!readRobotState())
    throw std::runtime_error("CartesianTask::update(): failed to read robot joint state");

  // --- Kinematics in base_link frame (OpenSoT behavior) ---
  if(!robot_.getJacobian(q_, distal_link_, base_link_, J_))
    throw std::runtime_error("CartesianTask::update(): getJacobian(q, distal, base) failed");

  if(!robot_.getPose(q_, distal_link_, base_link_, pose_current_))
    throw std::runtime_error("CartesianTask::update(): getPose(q, distal, base) failed");

  if(!robot_.getTwist(q_, qd_, distal_link_, base_link_, vel_current_))
    vel_current_.setZero();

  // TODO: when available implement relative JdotQdot like OpenSoT computeRelativeJdotQdot
  jdotqdot_.setZero();

  computeOrientationError(pose_ref_.linear(), pose_current_.linear(), orientation_error_);

  pose_error_.head<3>() = pose_ref_.translation() - pose_current_.translation();
  pose_error_.tail<3>() = orientation_gain_ * orientation_error_;

  vel_error_ = vel_ref_ - vel_current_;

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

  // OpenSoT style: A = J*qddot + jdotqdot - y  =>  A x = b with b = y - jdotqdot
  b_ = y - jdotqdot_;

  // consume refs (OpenSoT behavior)
  vel_ref_.setZero();
  acc_ref_.setZero();
  virtual_force_ref_.setZero();
}

} // namespace wolf_wbid
