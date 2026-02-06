#include <wolf_wbid/wbid/tasks/com_task.h>
#include <wolf_wbid/quadruped_robot.h>

#include <stdexcept>

namespace wolf_wbid {

ComTask::ComTask(const std::string& task_id,
                 QuadrupedRobot& robot,
                 const IDVariables& vars)
: TaskBase(task_id)
, robot_(robot)
, vars_(vars)
{
  qb_ = vars_.qddotBlock();
  if(qb_.dim <= 0) throw std::runtime_error("ComTask: invalid qddot block");

  resize(3, vars_.size());

  // default gains
  TaskBase::setKp(Eigen::Matrix3d::Identity());
  TaskBase::setKd(Eigen::Matrix3d::Identity());
}

void ComTask::setReference(const Eigen::Vector3d& p_ref, const Eigen::Vector3d& v_ref)
{
  p_ref_ = p_ref;
  v_ref_ = v_ref;
}

// ---- QuadrupedRobot adapters (adjust if needed) ----
void ComTask::getCOM(Eigen::Vector3d& p_W) const
{
  robot_.getCOM(p_W);
}
void ComTask::getCOMVelocity(Eigen::Vector3d& v_W) const
{
  robot_.getCOMVelocity(v_W);
}
void ComTask::getCOMJacobian(Eigen::MatrixXd& Jcom) const
{
  Jcom.resize(3, robot_.getJointNum());
  robot_.getCOMJacobian(Jcom);
}
void ComTask::getCOMJacobianDotTimesQdot(Eigen::Vector3d& Jdot_qdot) const
{
  Jdot_qdot.setZero();
  // plug if available: robot_.getCOMJacobianDotTimesQdot(Jdot_qdot);
}
// ---------------------------------------------------

void ComTask::update(const Eigen::VectorXd& /*x*/)
{
  if(!enabled()) {
    b_.setZero();
    return;
  }

  Eigen::MatrixXd Jcom;
  getCOMJacobian(Jcom);

  if(Jcom.cols() != qb_.dim)
    throw std::runtime_error("ComTask::update(): Jcom cols != qddot dim");

  A_.setZero();
  A_.block(0, qb_.offset, 3, qb_.dim) = Jcom;

  // actual
  getCOM(p_act_);
  getCOMVelocity(v_act_);

  // errors
  e_p_ = p_ref_ - p_act_;
  e_v_ = v_ref_ - v_act_;

  if(getKp().rows() != 3 || getKd().rows() != 3)
    throw std::runtime_error("ComTask::update(): Kp/Kd must be 3x3");

  Eigen::Vector3d pdd_des = getKp() * e_p_ + getKd() * e_v_;

  Eigen::Vector3d Jdot_qd;
  getCOMJacobianDotTimesQdot(Jdot_qd);

  b_ = pdd_des - Jdot_qd;
}

bool ComTask::reset()
{
  // reference := current com
  getCOM(p_ref_);
  v_ref_.setZero();
  return true;
}

} // namespace wolf_wbid
