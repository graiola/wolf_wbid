#include <wolf_wbid/wbid/tasks/postural_task.h>
#include <wolf_wbid/core/quadruped_robot.h>

#include <stdexcept>

namespace wolf_wbid {

PosturalTask::PosturalTask(const std::string& task_id,
                           QuadrupedRobot& robot,
                           const IDVariables& vars)
: TaskBase(task_id)
, robot_(robot)
, vars_(vars)
{
  qb_ = vars_.qddotBlock();
  n_ = qb_.dim;

  if(n_ <= 0) throw std::runtime_error("PosturalTask: invalid qddot dim");

  // LSQ: nj rows, nvars cols
  resize(n_, vars_.size());

  // A = [I, 0] on qddot block
  A_.setZero();
  A_.block(0, qb_.offset, n_, n_).setIdentity();

  joint_names_ = getJointNames();

  q_act_.setZero(n_);
  qd_act_.setZero(n_);
  q_ref_.setZero(n_);
  qd_ref_.setZero(n_);
  e_q_.setZero(n_);
  e_qd_.setZero(n_);

  // default gains
  TaskBase::setKp(Eigen::MatrixXd::Identity(n_, n_));
  TaskBase::setKd(Eigen::MatrixXd::Identity(n_, n_));
}

void PosturalTask::setWeightDiag(double w)
{
  if(!std::isfinite(w) || w < 0.0)
    throw std::runtime_error("PosturalTask::setWeightDiag(): invalid weight");

  // constant row weights = w
  Eigen::VectorXd wd = Eigen::VectorXd::Constant(n_, 1.0);
  TaskBase::setWeight(wd);       // user weights
  TaskBase::setWeightScalar(w);  // scalar multiplier
}

void PosturalTask::setReference(const Eigen::VectorXd& q_ref)
{
  if(q_ref.size() != n_)
    throw std::runtime_error("PosturalTask::setReference(q): size mismatch");
  q_ref_ = q_ref;
}

void PosturalTask::setReference(const Eigen::VectorXd& q_ref, const Eigen::VectorXd& qd_ref)
{
  setReference(q_ref);
  if(qd_ref.size() != n_)
    throw std::runtime_error("PosturalTask::setReference(q,qd): qd size mismatch");
  qd_ref_ = qd_ref;
}

void PosturalTask::setGains(const Eigen::MatrixXd& Kp, const Eigen::MatrixXd& Kd)
{
  if(Kp.rows() != n_ || Kp.cols() != n_ || Kd.rows() != n_ || Kd.cols() != n_)
    throw std::runtime_error("PosturalTask::setGains(): size mismatch");
  TaskBase::setKp(Kp);
  TaskBase::setKd(Kd);
}

// ---- QuadrupedRobot adapters (adjust if needed) ----
void PosturalTask::getJointPosition(Eigen::VectorXd& q) const
{
  q.resize(robot_.getJointNum());
  robot_.getJointPosition(q);
}
void PosturalTask::getJointVelocity(Eigen::VectorXd& qd) const
{
  qd.resize(robot_.getJointNum());
  robot_.getJointVelocity(qd);
}
std::vector<std::string> PosturalTask::getJointNames() const
{
  return robot_.getJointNames();
}
// ---------------------------------------------------

void PosturalTask::update(const Eigen::VectorXd& /*x*/)
{
  if(!enabled()) {
    b_.setZero();
    return;
  }

  // Read current joint state
  getJointPosition(q_act_);
  getJointVelocity(qd_act_);

  if(q_act_.size() != n_ || qd_act_.size() != n_)
    throw std::runtime_error("PosturalTask::update(): joint state size mismatch");

  // Errors
  e_q_  = q_ref_  - q_act_;
  e_qd_ = qd_ref_ - qd_act_;

  if(getKp().rows() != n_ || getKp().cols() != n_ ||
     getKd().rows() != n_ || getKd().cols() != n_)
    throw std::runtime_error("PosturalTask::update(): Kp/Kd size mismatch");

  // One-shot acceleration feedforward was present in the old stack; if you don't expose it
  // in the new API, keep it as zero here.
  const Eigen::VectorXd qdd_ref = Eigen::VectorXd::Zero(n_);

  // PD term (in joint space)
  const Eigen::VectorXd pd_term =
      getKd() * e_qd_ + getKp() * e_q_;

  Eigen::VectorXd qdd_des(n_);
  if(getGainType() == GainType::Acceleration)
  {
    // Acceleration-mode gains: directly shape desired joint acceleration.
    qdd_des = qdd_ref + pd_term;
  }
  else
  {
    // Force-mode gains: map the PD term through inverse inertia (as in the old implementation).
    Eigen::MatrixXd Mi;
    robot_.getInertiaInverse(Mi);
    if(Mi.rows() != n_ || Mi.cols() != n_)
      throw std::runtime_error("PosturalTask::update(): inertia inverse size mismatch");

    qdd_des = qdd_ref + Mi * pd_term;
  }

  // Task: qddot ~= qdd_des  ->  A x = b with A selecting qddot block
  b_ = qdd_des;

  // OpenSoT-like: if you want "one-shot" velocity reference, reset it here.
  // (Keep/remove depending on how your wrapper uses qd_ref_.)
  // qd_ref_.setZero(n_);
}


double PosturalTask::computeCost(const Eigen::VectorXd& x) const
{
  // generic LSQ cost: 0.5 (Ax-b)' W (Ax-b) with W = diag(wDiag)
  if(x.size() != cols()) return 0.0;
  const Eigen::VectorXd r = A_ * x - b_;
  const Eigen::VectorXd wd = wDiag();
  if(wd.size() != r.size()) return 0.0;
  return 0.5 * (r.array().square() * wd.array()).sum();
}

bool PosturalTask::reset()
{
  // reference := current q
  getJointPosition(q_ref_);
  qd_ref_.setZero(n_);
  return true;
}

} // namespace wolf_wbid
