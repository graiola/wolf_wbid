#include <wolf_wbid/wbid/tasks/cartesian_task.h>
#include <wolf_wbid/quadruped_robot.h>

#include <stdexcept>

namespace wolf_wbid {

CartesianTask::CartesianTask(const std::string& task_id,
                             QuadrupedRobot& robot,
                             const std::string& distal_link,
                             const std::string& base_link,
                             const IDVariables& vars)
: TaskBase(task_id)
, robot_(robot)
, vars_(vars)
, distal_link_(distal_link)
, base_link_(base_link)
{
  if(distal_link_.empty()) throw std::runtime_error("CartesianTask: empty distal_link");
  if(base_link_.empty())   throw std::runtime_error("CartesianTask: empty base_link");

  qb_ = vars_.qddotBlock();
  if(qb_.dim <= 0) throw std::runtime_error("CartesianTask: invalid qddot block");

  // LSQ: 6 rows, nvars cols
  resize(6, vars_.size());

  // default gains
  Eigen::Matrix<double,6,6> Kp = Eigen::Matrix<double,6,6>::Identity();
  Eigen::Matrix<double,6,6> Kd = Eigen::Matrix<double,6,6>::Identity();
  TaskBase::setKp(Kp);
  TaskBase::setKd(Kd);

  // default reference: current (on reset)
  pose_ref_.setIdentity();
  twist_ref_.setZero();
}

bool CartesianTask::setBaseLink(const std::string& new_base_link)
{
  if(new_base_link.empty()) return false;
  base_link_ = new_base_link;
  return true;
}

void CartesianTask::setReference(const Eigen::Affine3d& pose_ref,
                                 const Eigen::Matrix<double,6,1>& twist_ref)
{
  pose_ref_ = pose_ref;
  twist_ref_ = twist_ref;
}

Eigen::Matrix<double,6,1> CartesianTask::se3LogApprox(const Eigen::Affine3d& T_err)
{
  // Lightweight SE(3) error:
  // translational part = p
  // rotational part = angle-axis vector
  Eigen::Matrix<double,6,1> e; e.setZero();
  e.head<3>() = T_err.translation();

  Eigen::AngleAxisd aa(T_err.linear());
  Eigen::Vector3d w = aa.axis() * aa.angle();
  if(!std::isfinite(w.x()) || !std::isfinite(w.y()) || !std::isfinite(w.z())) w.setZero();
  e.tail<3>() = w;
  return e;
}

// ---- QuadrupedRobot API adapters (adjust if needed) ----
void CartesianTask::getJacobian6(const std::string& link, Eigen::MatrixXd& J6) const
{
  // expected: 6 x nj
  J6.resize(6, robot_.getJointNum());
  robot_.getJacobian(link, J6);
}

void CartesianTask::getJacobianDotTimesQdot6(const std::string& /*link*/,
                                            Eigen::Matrix<double,6,1>& Jdot_qdot) const
{
  // If you have Jdot*qd available, plug it here.
  // Otherwise keep zero (works, but slightly less accurate).
  Jdot_qdot.setZero();
}

void CartesianTask::getLinkPoseInWorld(const std::string& link, Eigen::Affine3d& T_W_L) const
{
  static constexpr const char* kWorldName = "world";

  // If link is already "world", pose is identity.
  if(link == kWorldName)
  {
    T_W_L.setIdentity();
    return;
  }

  // Use the existing QuadrupedRobot API (same used in the old implementation)
  if(!robot_.getPose(link, T_W_L))
  {
    T_W_L.setIdentity();
    throw std::runtime_error("CartesianTask::getLinkPoseInWorld(): getPose(link, world) failed for link=" + link);
  }
}

// --------------------------------------------------------

void CartesianTask::update(const Eigen::VectorXd& /*x*/)
{
  if(!enabled())
  {
    b_.setZero();
    return;
  }

  static constexpr const char* kWorldName = "world";

  // --- Read current pose / Jacobian / twist / Jdot*qdot using the same APIs as the old code
  Eigen::Affine3d pose_current = Eigen::Affine3d::Identity();
  Eigen::MatrixXd J6;                 // 6 x nj
  Eigen::Matrix<double,6,1> vel_current; vel_current.setZero();
  Eigen::Matrix<double,6,1> jdotqdot;     jdotqdot.setZero();

  const Eigen::Vector3d p_ref = Eigen::Vector3d::Zero();

  if(base_link_ == kWorldName)
  {
    if(!robot_.getPose(distal_link_, pose_current))
      throw std::runtime_error("CartesianTask::update(): getPose(distal, world) failed");

    if(!robot_.getJacobian(distal_link_, J6))
      throw std::runtime_error("CartesianTask::update(): getJacobian(distal, world) failed");

    if(!robot_.getVelocityTwist(distal_link_, vel_current))
      vel_current.setZero();

    if(!robot_.computeJdotQdot(distal_link_, p_ref, jdotqdot))
      jdotqdot.setZero();
  }
  else
  {
    if(!robot_.getPose(distal_link_, base_link_, pose_current))
      throw std::runtime_error("CartesianTask::update(): getPose(distal, base) failed");

    if(!robot_.getJacobian(distal_link_, base_link_, J6))
      throw std::runtime_error("CartesianTask::update(): getJacobian(distal in base) failed");

    if(!robot_.getVelocityTwist(distal_link_, base_link_, vel_current))
      vel_current.setZero();

    if(!robot_.computeRelativeJdotQdot(distal_link_, base_link_, jdotqdot))
      jdotqdot.setZero();
  }

  if(J6.cols() != qb_.dim)
    throw std::runtime_error("CartesianTask::update(): Jacobian cols != qddot dim");

  // --- Build A = [J, 0] on the qddot block
  A_.setZero();
  A_.block(0, qb_.offset, 6, qb_.dim) = J6;

  // --- Pose error (OpenSoT-style): position + small-angle orientation
  Eigen::Matrix<double,6,1> pose_error; pose_error.setZero();
  pose_error.head<3>() = pose_ref_.translation() - pose_current.translation();

  const Eigen::Matrix3d R_des = pose_ref_.linear();
  const Eigen::Matrix3d R_act = pose_current.linear();
  const Eigen::Matrix3d R_err = R_des * R_act.transpose();

  Eigen::Vector3d vee;
  vee << (R_err(2,1) - R_err(1,2)),
         (R_err(0,2) - R_err(2,0)),
         (R_err(1,0) - R_err(0,1));

  // orientation_gain_ was a member in the old code; keep default gain=1.0 here
  pose_error.tail<3>() = 0.5 * vee;

  // --- Velocity error
  const Eigen::Matrix<double,6,1> vel_error = twist_ref_ - vel_current;

  if(getKp().rows() != 6 || getKp().cols() != 6 || getKd().rows() != 6 || getKd().cols() != 6)
    throw std::runtime_error("CartesianTask::update(): Kp/Kd must be 6x6");

  // Feedforward terms existed in the old implementation (acc_ref_, virtual_force_ref_).
  // If your current header does not store them, treat them as zero (still stable/consistent).
  const Eigen::Matrix<double,6,1> acc_ref = Eigen::Matrix<double,6,1>::Zero();
  const Eigen::Matrix<double,6,1> f_virtual = Eigen::Matrix<double,6,1>::Zero();

  Eigen::Matrix<double,6,1> y; y.setZero();

  if(getGainType() == GainType::Acceleration)
  {
    // Acceleration mode: y = acc_ref + lambda2 * Kd * e_v + lambda1 * Kp * e_x
    y = acc_ref
        + getLambda2() * (getKd() * vel_error)
        + getLambda1() * (getKp() * pose_error);
  }
  else
  {
    // Force mode: y = acc_ref + Mi*(lambda2*Kd*e_v + lambda1*Kp*e_x + f_virtual)
    Eigen::MatrixXd Bi;
    robot_.getInertiaInverse(Bi);

    if(Bi.rows() != qb_.dim || Bi.cols() != qb_.dim)
      throw std::runtime_error("CartesianTask::update(): inertia inverse size mismatch");

    Eigen::MatrixXd tmp6xn = J6 * Bi;                 // 6 x n
    Eigen::Matrix<double,6,6> Mi = tmp6xn * J6.transpose();

    y = acc_ref
        + getLambda2() * (Mi * (getKd() * vel_error))
        + getLambda1() * (Mi * (getKp() * pose_error))
        + Mi * f_virtual;
  }

  // Final constraint: J*qddot = y - Jdot*qdot
  b_ = y - jdotqdot;
}

bool CartesianTask::reset()
{
  // set reference to current pose
  Eigen::Affine3d T_W_E, T_W_B;
  getLinkPoseInWorld(distal_link_, T_W_E);
  getLinkPoseInWorld(base_link_,   T_W_B);
  pose_ref_ = T_W_B.inverse() * T_W_E;

  twist_ref_.setZero();
  return true;
}

} // namespace wolf_wbid
