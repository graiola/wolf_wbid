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
  qdd_ref_.setZero(n_);
  pd_term_.setZero(n_);
  qdd_des_.setZero(n_);
  Mi_.setZero(n_, n_);

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
  if(q_ref.size() == n_) {
    q_ref_ = q_ref;
  } else if(q_ref.size() == n_ + 1) {
    q_ref_.setZero(n_);
    q_ref_.tail(n_ - 6) = q_ref.tail(n_ - 6);
  } else if(q_ref.size() == n_ - 6) {
    q_ref_.setZero(n_);
    q_ref_.tail(n_ - 6) = q_ref;
  } else {
    throw std::runtime_error("PosturalTask::setReference(q): size mismatch: q_ref.size()=" +
                             std::to_string(q_ref.size()) + " n_=" + std::to_string(n_));
  }
}

void PosturalTask::setReference(const Eigen::VectorXd& q_ref, const Eigen::VectorXd& qd_ref)
{
  setReference(q_ref);
  if(qd_ref.size() == n_) {
    qd_ref_ = qd_ref;
  } else if(qd_ref.size() == n_ - 6) {
    qd_ref_.setZero(n_);
    qd_ref_.tail(n_ - 6) = qd_ref;
  } else {
    throw std::runtime_error("PosturalTask::setReference(q,qd): qd size mismatch: qd_ref.size()=" +
                             std::to_string(qd_ref.size()) + " n_=" + std::to_string(n_));
  }
}

void PosturalTask::setGains(const Eigen::MatrixXd& Kp, const Eigen::MatrixXd& Kd)
{
  if(Kp.rows() == n_ && Kp.cols() == n_ && Kd.rows() == n_ && Kd.cols() == n_) {
    TaskBase::setKp(Kp);
    TaskBase::setKd(Kd);
  } else if(Kp.rows() == n_ - 6 && Kp.cols() == n_ - 6 && Kd.rows() == n_ - 6 && Kd.cols() == n_ - 6) {
    Eigen::MatrixXd Kp_full = Eigen::MatrixXd::Zero(n_, n_);
    Eigen::MatrixXd Kd_full = Eigen::MatrixXd::Zero(n_, n_);
    Kp_full.bottomRightCorner(n_ - 6, n_ - 6) = Kp;
    Kd_full.bottomRightCorner(n_ - 6, n_ - 6) = Kd;
    TaskBase::setKp(Kp_full);
    TaskBase::setKd(Kd_full);
  } else {
    throw std::runtime_error("PosturalTask::setGains(): size mismatch: Kp.rows()=" +
                             std::to_string(Kp.rows()) + " n_=" + std::to_string(n_));
  }
}

// ---- QuadrupedRobot adapters (adjust if needed) ----
void PosturalTask::getJointPosition(Eigen::VectorXd& q) const
{
  robot_.getJointPosition(q);
}
void PosturalTask::getJointVelocity(Eigen::VectorXd& qd) const
{
  robot_.getJointVelocity(qd);
}
std::vector<std::string> PosturalTask::getJointNames() const
{
  return robot_.getJointNames();
}
// ---------------------------------------------------

void PosturalTask::update()
{
  if(!enabled()) {
    b_.setZero();
    return;
  }

  // Read current joint state
  getJointPosition(q_act_);
  getJointVelocity(qd_act_);

  if(q_act_.size() != n_) {
    if(q_act_.size() == n_ + 1) {
      Eigen::VectorXd tmp = Eigen::VectorXd::Zero(n_);
      tmp.tail(n_ - 6) = q_act_.tail(n_ - 6);
      q_act_ = tmp;
    } else if(q_act_.size() == n_ - 6) {
      Eigen::VectorXd tmp = Eigen::VectorXd::Zero(n_);
      tmp.tail(n_ - 6) = q_act_;
      q_act_ = tmp;
    } else {
      throw std::runtime_error("PosturalTask::update(): q_act size mismatch: q_act.size()=" +
                               std::to_string(q_act_.size()) + " n_=" + std::to_string(n_));
    }
  }

  if(qd_act_.size() != n_) {
    if(qd_act_.size() == n_ - 6) {
      Eigen::VectorXd tmp = Eigen::VectorXd::Zero(n_);
      tmp.tail(n_ - 6) = qd_act_;
      qd_act_ = tmp;
    } else {
      throw std::runtime_error("PosturalTask::update(): qd_act size mismatch: qd_act.size()=" +
                               std::to_string(qd_act_.size()) + " n_=" + std::to_string(n_));
    }
  }

  // Errors
  e_q_  = q_ref_  - q_act_;
  e_qd_ = qd_ref_ - qd_act_;

  if(n_ > 6) {
    e_q_.head(6).setZero();
    e_qd_.head(6).setZero();
  }

  if(getKp().rows() != n_ || getKp().cols() != n_ ||
     getKd().rows() != n_ || getKd().cols() != n_)
    throw std::runtime_error("PosturalTask::update(): Kp/Kd size mismatch");

  // One-shot acceleration feedforward was present in the old stack; if you don't expose it
  // in the new API, keep it as zero here.
  qdd_ref_.setZero();

  // PD term (in joint space)
  pd_term_.noalias() = getKd() * e_qd_ + getKp() * e_q_;

  if(getGainType() == GainType::Acceleration)
  {
    // Acceleration-mode gains: directly shape desired joint acceleration.
    qdd_des_.noalias() = qdd_ref_ + pd_term_;
  }
  else
  {
    // Force-mode gains: map the PD term through inverse inertia (as in the old implementation).
    robot_.getInertiaInverse(Mi_);
    if(Mi_.rows() != n_ || Mi_.cols() != n_)
      throw std::runtime_error("PosturalTask::update(): inertia inverse size mismatch");

    qdd_des_.noalias() = qdd_ref_ + Mi_ * pd_term_;
  }

  for(int k=0; k<n_; ++k) {
    if(!std::isfinite(qdd_des_(k)) || std::abs(qdd_des_(k)) > 15.0) {
      qdd_des_(k) = std::clamp(qdd_des_(k), -15.0, 15.0);
    }
  }

  // Task: qddot ~= qdd_des  ->  A x = b with A selecting qddot block
  b_ = qdd_des_;

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
  if(q_ref_.size() != n_) {
    if(q_ref_.size() == n_ + 1) {
      Eigen::VectorXd tmp = Eigen::VectorXd::Zero(n_);
      tmp.tail(n_ - 6) = q_ref_.tail(n_ - 6);
      q_ref_ = tmp;
    } else if(q_ref_.size() == n_ - 6) {
      Eigen::VectorXd tmp = Eigen::VectorXd::Zero(n_);
      tmp.tail(n_ - 6) = q_ref_;
      q_ref_ = tmp;
    }
  }
  qd_ref_.setZero(n_);
  return true;
}

} // namespace wolf_wbid
