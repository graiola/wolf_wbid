// ============================================================================
// File: src/wbid/tasks/cartesian_task.cpp
// CartesianTask: OpenSoT-like behavior using ONLY ModelInterface Eigen APIs
// ============================================================================

#include <wolf_wbid/wbid/tasks/cartesian_task.h>

#include <wolf_wbid/wbid/id_variables.h>
#include <wolf_wbid/quadruped_robot.h>

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace wolf_wbid {

static constexpr const char* kWorldName = "world";

// -----------------------------
// Constructor
// -----------------------------
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

  A_.setZero(6, vars_.size());
  b_.setZero(6);

  q_.setZero(n);
  qd_.setZero(n);

  // OpenSoT-ish defaults
  orientation_gain_ = 1.0;
  gain_type_ = GainType::Acceleration;

  lambda1_ = 100.0;
  lambda2_ = 2.0 * std::sqrt(lambda1_);

  Kp_.setIdentity();
  Kd_.setIdentity();

  vel_ref_.setZero();
  acc_ref_.setZero();
  virtual_force_ref_.setZero();

  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
  virtual_force_ref_cached_ = virtual_force_ref_;

  pose_current_.setIdentity();
  pose_ref_.setIdentity();

  pose_error_.setZero();
  vel_error_.setZero();
  vel_current_.setZero();
  jdotqdot_.setZero();

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

  // Keep reference consistent when changing base
  Eigen::Affine3d oldBase_T_newBase = Eigen::Affine3d::Identity();

  // Use MODEL (not q-explicit) to get pose between frames, like OpenSoT would
  if(!robot_.getPose(base_link_, base_link, oldBase_T_newBase))
    return false;

  pose_ref_ = oldBase_T_newBase * pose_ref_;
  base_link_ = base_link;
  return true;
}

bool CartesianTask::setDistalLink(const std::string& distal_link)
{
  if(distal_link == distal_link_) return true;
  if(distal_link == kWorldName) return false;

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
  // reference = current pose (OpenSoT behavior)
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();

  if(base_link_ == kWorldName)
  {
    if(!robot_.getPose(distal_link_, pose))
      pose.setIdentity();
  }
  else
  {
    if(!robot_.getPose(distal_link_, base_link_, pose))
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

// small-angle orientation error (OpenSoT-style)
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
  robot_.getInertiaInverse(Bi_);
  tmp6xn_.noalias() = J_ * Bi_;
  Mi_.noalias()     = tmp6xn_ * J_.transpose();
}

// -----------------------------
// UPDATE (core)
// -----------------------------
void CartesianTask::update(const Eigen::VectorXd& x)
{
  // cache refs (OpenSoT behavior)
  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
  virtual_force_ref_cached_ = virtual_force_ref_;

  // ensure internal state is readable (q, qd not used for kinematics here)
  if(!readRobotState())
    throw std::runtime_error("CartesianTask::update(): failed to read joint state");

  // IMPORTANT: we rely on the model internal state (already updated in the control loop).
  // If you suspect update() is not called elsewhere, uncomment this:
  // robot_.update(true, true, true);

  // ------------------------------------------------------------
  // Kinematics/Differential kinematics ONLY via ModelInterface Eigen overloads
  // ------------------------------------------------------------
  const Eigen::Vector3d p_ref = Eigen::Vector3d::Zero();

  if(base_link_ == kWorldName)
  {
    if(!robot_.getPose(distal_link_, pose_current_))
      throw std::runtime_error("CartesianTask::update(): getPose(distal, world) failed");

    if(!robot_.getJacobian(distal_link_, J_))
      throw std::runtime_error("CartesianTask::update(): getJacobian(distal, world) failed");

    if(!robot_.getVelocityTwist(distal_link_, vel_current_))
      vel_current_.setZero();

    if(!robot_.computeJdotQdot(distal_link_, p_ref, jdotqdot_))
      jdotqdot_.setZero();
  }
  else
  {
    if(!robot_.getPose(distal_link_, base_link_, pose_current_))
      throw std::runtime_error("CartesianTask::update(): getPose(distal, base) failed");

    // expressed in base frame (this is what OpenSoT expects for relative tasks)
    if(!robot_.getJacobian(distal_link_, base_link_, J_))
      throw std::runtime_error("CartesianTask::update(): getJacobian(distal in base) failed");

    if(!robot_.getVelocityTwist(distal_link_, base_link_, vel_current_))
      vel_current_.setZero();

    if(!robot_.computeRelativeJdotQdot(distal_link_, base_link_, jdotqdot_))
      jdotqdot_.setZero();
  }

  // ------------------------------------------------------------
  // Errors + tracking law (OpenSoT-like)
  // ------------------------------------------------------------
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

  // ------------------------------------------------------------
  // Constraint: J*qddot = y - Jdot*qdot
  // ------------------------------------------------------------
  A_.setZero();
  const auto& qb = vars_.qddotBlock();
  A_.block(0, qb.offset, 6, qb.dim) = J_;

  b_ = y - jdotqdot_;

  // OpenSoT: one-shot feedforward reset
  vel_ref_.setZero();
  acc_ref_.setZero();
  virtual_force_ref_.setZero();
}

} // namespace wolf_wbid
