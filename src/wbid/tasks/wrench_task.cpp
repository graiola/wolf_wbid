#include <wolf_wbid/wbid/tasks/wrench_task.h>

#include <stdexcept>

namespace wolf_wbid {

WrenchTask::WrenchTask(const std::string& task_id,
                       const std::string& contact_name,
                       const IDVariables& vars,
                       double weight_scalar)
: TaskBase(task_id)
, contact_name_(contact_name)
, vars_(vars)
{
  if(contact_name_.empty())
    throw std::runtime_error("WrenchTask: empty contact_name");

  if(!vars_.hasContact(contact_name_))
    throw std::runtime_error("WrenchTask: unknown contact: " + contact_name_);

  cb_ = vars_.contactBlock(contact_name_);

  // Expect point contact for force tracking (3 dims).
  if(cb_.dim < 3)
    throw std::runtime_error("WrenchTask: contact block dim < 3 for " + contact_name_);

  // LSQ is 3 rows (Fx,Fy,Fz), cols = nvars
  resize(3, vars_.size());

  // Selection matrix: pick [Fx,Fy,Fz] from the contact block
  // A = [ 0 ... I3(at cb_.offset) ... 0 ]
  A_.setZero();
  A_.block(0, cb_.offset, 3, 3).setIdentity();
  A_base_ = A_;

  // Default weights: ones * weight_scalar
  setWeightScalar(weight_scalar);

  // Default "lambda-like" gain
  setLambda(1.0, 0.0);

  // Reference default
  b_.setZero();
  f_ref_.setZero();
}

void WrenchTask::setReference(const Eigen::Vector3d& f_ref)
{
  if(!std::isfinite(f_ref.x()) || !std::isfinite(f_ref.y()) || !std::isfinite(f_ref.z()))
    throw std::runtime_error("WrenchTask::setReference(): non-finite reference");

  f_ref_ = f_ref;
  b_ = f_ref_;
}

void WrenchTask::update(const Eigen::VectorXd& /*x*/)
{
  // A is a scaled selector, b is the scaled reference.
  // (No need to read x here; IDProblem uses A,b,wDiag to build H,g.)
  // Use lambda1 as an extra gain (legacy behavior).
  double gain = getLambda1();
  if(!(gain > 0.0)) gain = 1.0;

  A_ = gain * A_base_;
  b_ = gain * f_ref_;
}

bool WrenchTask::reset()
{
  f_ref_.setZero();
  b_.setZero();
  A_ = A_base_;
  return true;
}

} // namespace wolf_wbid
