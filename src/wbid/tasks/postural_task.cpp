#include <wolf_wbid/wbid/tasks/postural_task.h>

namespace wolf_wbid {

static void _check_size(const Eigen::VectorXd& v, int n, const std::string& what)
{
  if(v.size() != n)
    throw std::runtime_error(what + " has wrong size: " + std::to_string(v.size()) +
                             " expected " + std::to_string(n));
}

PosturalTask::PosturalTask(const std::string& task_id,
                           QuadrupedRobot& model,
                           const IDVariables& idvars,
                           double period)
: task_id_(task_id)
, model_(model)
, idvars_(idvars)
, period_(period)
{
  if(period_ <= 0.0) throw std::runtime_error("PosturalTask: period must be > 0");

  task_size_ = model_.getJointNum();
  x_size_    = idvars_.size();

  joint_names_ = model_.getJointNames();

  // allocate vectors
  q_.setZero(task_size_);
  qdot_.setZero(task_size_);
  qref_.setZero(task_size_);
  qdot_ref_.setZero(task_size_);
  qddot_ref_.setZero(task_size_);
  qdot_ref_cached_.setZero(task_size_);
  qddot_ref_cached_.setZero(task_size_);

  position_error_.setZero(task_size_);
  velocity_error_.setZero(task_size_);
  qddot_d_.setZero(task_size_);

  // gains
  Kp_.setIdentity(task_size_, task_size_);
  Kd_.setIdentity(task_size_, task_size_);

  // task matrices
  buildSelectionMatrix();
  b_.setZero(task_size_);
  setWeightDiag(weight_diag_);

  // initialize reference to current posture if possible
  reset();
}

void PosturalTask::buildSelectionMatrix()
{
  // A selects qddot block from x: A x = qddot
  // A is (nq x x_size) with I on qddot offset
  A_.setZero(task_size_, x_size_);

  const int off = 0; // in your IDVariables implementation qddot is first, offset 0.
  // If in future it changes, use idvars_.qddotOffset() accessor.
  if(off + task_size_ > x_size_)
    throw std::runtime_error("PosturalTask: invalid qddot block layout");

  A_.block(0, off, task_size_, task_size_) = Eigen::MatrixXd::Identity(task_size_, task_size_);
}

void PosturalTask::setWeightDiag(double w)
{
  if(w < 0.0) throw std::runtime_error("PosturalTask: weight must be >= 0");
  weight_diag_ = w;
  W_.setIdentity(task_size_, task_size_);
  W_ *= weight_diag_;
}

void PosturalTask::setLambda(double lambda1)
{
  if(lambda1 < 0.0) throw std::runtime_error("PosturalTask: lambda1 must be >= 0");
  lambda1_ = lambda1;
  lambda2_ = 2.0 * std::sqrt(std::max(0.0, lambda1_));
}

void PosturalTask::setLambda(double lambda1, double lambda2)
{
  if(lambda1 < 0.0 || lambda2 < 0.0)
    throw std::runtime_error("PosturalTask: lambdas must be >= 0");
  lambda1_ = lambda1;
  lambda2_ = lambda2;
}

void PosturalTask::setKp(const Eigen::MatrixXd& Kp)
{
  if(Kp.rows() != task_size_ || Kp.cols() != task_size_)
    throw std::runtime_error("PosturalTask: Kp wrong size");
  Kp_ = Kp;
}

void PosturalTask::setKd(const Eigen::MatrixXd& Kd)
{
  if(Kd.rows() != task_size_ || Kd.cols() != task_size_)
    throw std::runtime_error("PosturalTask: Kd wrong size");
  Kd_ = Kd;
}

void PosturalTask::setGains(const Eigen::MatrixXd& Kp, const Eigen::MatrixXd& Kd)
{
  setKp(Kp);
  setKd(Kd);
}

void PosturalTask::setReference(const Eigen::VectorXd& qref)
{
  _check_size(qref, task_size_, "PosturalTask::setReference(qref)");
  qref_ = qref;
  qdot_ref_.setZero(task_size_);
  qddot_ref_.setZero(task_size_);
  qdot_ref_cached_ = qdot_ref_;
  qddot_ref_cached_ = qddot_ref_;
}

void PosturalTask::setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& qdot_ref)
{
  _check_size(qref, task_size_, "PosturalTask::setReference(qref,qdot)");
  _check_size(qdot_ref, task_size_, "PosturalTask::setReference(qref,qdot)");
  qref_ = qref;
  qdot_ref_ = qdot_ref;
  qddot_ref_.setZero(task_size_);
  qdot_ref_cached_ = qdot_ref_;
  qddot_ref_cached_ = qddot_ref_;
}

void PosturalTask::setReference(const Eigen::VectorXd& qref,
                                const Eigen::VectorXd& qdot_ref,
                                const Eigen::VectorXd& qddot_ref)
{
  _check_size(qref, task_size_, "PosturalTask::setReference(qref,qdot,qddot)");
  _check_size(qdot_ref, task_size_, "PosturalTask::setReference(qref,qdot,qddot)");
  _check_size(qddot_ref, task_size_, "PosturalTask::setReference(qref,qdot,qddot)");
  qref_ = qref;
  qdot_ref_ = qdot_ref;
  qddot_ref_ = qddot_ref;
  qdot_ref_cached_ = qdot_ref_;
  qddot_ref_cached_ = qddot_ref_;
}

bool PosturalTask::reset()
{
  // set qref = current q
  model_.getJointPosition(q_);
  model_.getJointVelocity(qdot_);
  qref_ = q_;
  qdot_ref_.setZero(task_size_);
  qddot_ref_.setZero(task_size_);
  qdot_ref_cached_ = qdot_ref_;
  qddot_ref_cached_ = qddot_ref_;
  return true;
}

void PosturalTask::update(const Eigen::VectorXd& x)
{
  _update(x);
  cost_last_ = computeCost(x);
}

void PosturalTask::_update(const Eigen::VectorXd& x)
{
  (void)x;

  // cache refs (published)
  qdot_ref_cached_  = qdot_ref_;
  qddot_ref_cached_ = qddot_ref_;

  // read state
  model_.getJointPosition(q_);
  model_.getJointVelocity(qdot_);

  // errors
  position_error_ = (qref_ - q_);
  velocity_error_ = (qdot_ref_ - qdot_);

  // desired qddot
  if(gain_type_ == GainType::Acceleration)
  {
    qddot_d_ = qddot_ref_ + lambda2_ * (Kd_ * velocity_error_) + lambda1_ * (Kp_ * position_error_);
  }
  else // GainType::Force
  {
    model_.getInertiaInverse(Mi_);
    if(Mi_.rows() != task_size_ || Mi_.cols() != task_size_)
      throw std::runtime_error("PosturalTask: Mi wrong size from model.getInertiaInverse()");
    const Eigen::VectorXd tmp = lambda2_ * (Kd_ * velocity_error_) + lambda1_ * (Kp_ * position_error_);
    qddot_d_ = qddot_ref_ + Mi_ * tmp;
  }

  // Build b so that Ax - b = qddot - qddot_d
  b_ = qddot_d_;

  // reset one-shot refs like OpenSoT
  qdot_ref_.setZero(task_size_);
  qddot_ref_.setZero(task_size_);
}

double PosturalTask::computeCost(const Eigen::VectorXd& x) const
{
  if(x.size() != x_size_) return 0.0;

  const Eigen::VectorXd r = (A_ * x) - b_;        // qddot - qddot_d
  const double c = 0.5 * r.transpose() * W_ * r;
  return c;
}

} // namespace wolf_wbid
