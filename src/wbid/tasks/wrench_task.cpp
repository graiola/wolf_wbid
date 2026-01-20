#include <wolf_wbid/wbid/tasks/wrench_task.h>
#include <wolf_wbid/wbid/id_variables.h>

#include <stdexcept>

namespace wolf_wbid {

WrenchTask::WrenchTask(std::string task_id,
                       std::string contact_name,
                       double weight)
: task_id_(std::move(task_id))
, contact_name_(std::move(contact_name))
, weight_(weight)
{
  if(weight < 0.0) {
    throw std::runtime_error("WrenchTask: negative weight");
  }
}

void WrenchTask::setWeight(double w)
{
  if(w < 0.0) {
    throw std::runtime_error("WrenchTask: negative weight");
  }
  weight_.store(w);
}

void WrenchTask::setReference(const Eigen::Vector3d& f_ref)
{
  f_ref_ = f_ref;
}

bool WrenchTask::reset()
{
  f_ref_.setZero();
  return true;
}

void WrenchTask::compute(const IDVariables& vars, LsqTerm& out)
{
  // Only POINT_CONTACT supported (R^3)
  if(vars.contactDim() != 3) {
    throw std::runtime_error("WrenchTask: only POINT_CONTACT (3D force) supported");
  }

  if(!vars.hasContact(contact_name_)) {
    throw std::runtime_error("WrenchTask: unknown contact '" + contact_name_ + "'");
  }

  constexpr int cd = 3;
  const int n = vars.size();
  const int off = vars.contactOffset(contact_name_);

  if(off < 0 || off + cd > n) {
    throw std::runtime_error("WrenchTask: contact block out of range for '" + contact_name_ + "'");
  }

  // A selects the contact force block from x
  out.A.setZero(cd, n);
  out.A.block(0, off, cd, cd).setIdentity();

  // b is the desired force reference
  out.b = f_ref_;

  // diagonal weights (same scalar replicated on 3 axes)
  out.w_diag.setConstant(cd, weight_.load());
}

void wolf_wbid::WrenchTask::update(const Eigen::VectorXd& /*x*/)
{
  // No-op:
  // For WrenchTask the LSQ term (A,b,W) does not depend on x.
  // Reference/weight are handled via setters (called by wrappers).
}

} // namespace wolf_wbid
