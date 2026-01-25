// wolf_wbid/wbid/constraints/torque_limits_constraint.cpp
#include <wolf_wbid/wbid/constraints/torque_limits_constraint.h>

#include <algorithm>
#include <stdexcept>

namespace wolf_wbid {

TorqueLimitsConstraint::TorqueLimitsConstraint(const std::string& name,
                                               QuadrupedRobot& robot,
                                               const IDVariables& vars,
                                               const std::vector<std::string>& contact_links,
                                               const Eigen::VectorXd& torque_limits)
: ConstraintBase(name),
  robot_(robot),
  vars_(vars),
  contacts_(contact_links),
  tau_lim_(torque_limits)
{
  enabled_.assign(contacts_.size(), true);

  // We constrain all robot dofs torques (including floating base if present).
  // In your old pipeline you set base limits to zero; you can keep that in tau_lim_.
  if(tau_lim_.size() != robot_.getJointNum())
    throw std::runtime_error("TorqueLimitsConstraint: torque_limits size != robot.getJointNum()");

  // m = ndofs, n = vars.size()
  resizeLinear(robot_.getJointNum(), vars_.size());
}

int TorqueLimitsConstraint::idxOf(const std::string& contact_link) const
{
  auto it = std::find(contacts_.begin(), contacts_.end(), contact_link);
  if(it == contacts_.end()) return -1;
  return static_cast<int>(std::distance(contacts_.begin(), it));
}

bool TorqueLimitsConstraint::enableContact(const std::string& contact_link)
{
  int i = idxOf(contact_link);
  if(i < 0) return false;
  enabled_[static_cast<size_t>(i)] = true;
  return true;
}

bool TorqueLimitsConstraint::disableContact(const std::string& contact_link)
{
  int i = idxOf(contact_link);
  if(i < 0) return false;
  enabled_[static_cast<size_t>(i)] = false;
  return true;
}

void TorqueLimitsConstraint::setTorqueLimits(const Eigen::VectorXd& tau_lim)
{
  if(tau_lim.size() != tau_lim_.size())
    throw std::runtime_error("TorqueLimitsConstraint::setTorqueLimits(): wrong size");
  tau_lim_ = tau_lim;
}

/*void TorqueLimitsConstraint::update(const Eigen::VectorXd& x)
{
  // Get inertia and nonlinear term
  robot_.getInertiaMatrix(B_);
  robot_.computeNonlinearTerm(h_);

  const int ndofs = robot_.getJointNum();
  if(B_.rows() != ndofs || B_.cols() != ndofs || h_.size() != ndofs)
    throw std::runtime_error("TorqueLimitsConstraint: inconsistent robot dynamics sizes");

  // A*x = B*qddot - sum J^T * w
  A_.setZero();

  // qddot block
  const auto& qb = vars_.qddotBlock();
  if(qb.dim != ndofs)
    throw std::runtime_error("TorqueLimitsConstraint: vars.qddotBlock().dim != robot dofs");

  A_.block(0, qb.offset, ndofs, qb.dim) = B_;

  // contacts contribution
  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    if(!enabled_[i]) continue;

    const std::string& c = contacts_[i];
    if(!vars_.hasContact(c)) continue;

    robot_.getJacobian(c, Jtmp_); // 6 x ndofs (expected)
    if(Jtmp_.cols() != ndofs || Jtmp_.rows() < 3)
      throw std::runtime_error("TorqueLimitsConstraint: bad Jacobian size for contact " + c);

    const auto& cb = vars_.contactBlock(c);

    if(vars_.contactDim() == 3)
    {
      // POINT_CONTACT: tau += -Jlin^T f
      // Jlin = topRows(3)
      A_.block(0, cb.offset, ndofs, 3).noalias() += -Jtmp_.topRows(3).transpose();
    }
    else if(vars_.contactDim() == 6)
    {
      // SURFACE_CONTACT: tau += -J^T w
      A_.block(0, cb.offset, ndofs, 6).noalias() += -Jtmp_.transpose();
    }
    else
    {
      throw std::runtime_error("TorqueLimitsConstraint: unsupported contactDim()");
    }
  }

  // bounds: -tau_lim <= (B*qddot - sum J^T w) + h <= tau_lim
  // equivalently: -tau_lim - h <= A*x <= tau_lim - h
  lA_ = (-tau_lim_ - h_);
  uA_ = ( tau_lim_ - h_);
}*/

void TorqueLimitsConstraint::update(const Eigen::VectorXd&)
{
  robot_.getInertiaMatrix(B_);
  robot_.computeNonlinearTerm(h_);

  const int ndofs = robot_.getJointNum();
  const int fb = robot_.isFloatingBase() ? 6 : 0;
  const int na = ndofs - fb;

  // vincolo solo sugli attuati
  resizeLinear(na, vars_.size());
  A_.setZero();

  const auto& qb = vars_.qddotBlock();
  A_.block(0, qb.offset + fb, na, na) = B_.block(fb, fb, na, na);

  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    if(!enabled_[i]) continue;
    const std::string& c = contacts_[i];
    if(!vars_.hasContact(c)) continue;

    robot_.getJacobian(c, Jtmp_);

    const auto& cb = vars_.contactBlock(c);

    if(vars_.contactDim() == 3)
    {
      // usa solo parte lineare, e solo righe attuate
      A_.block(0, cb.offset, na, 3).noalias() += -Jtmp_.topRows(3).transpose().block(fb, 0, na, 3);
    }
    else
    {
      A_.block(0, cb.offset, na, 6).noalias() += -Jtmp_.transpose().block(fb, 0, na, 6);
    }
  }

  // bounds su tau attuati
  const Eigen::VectorXd tau_lim_a = tau_lim_.tail(na);
  const Eigen::VectorXd h_a       = h_.tail(na);

  lA_ = (-tau_lim_a - h_a);
  uA_ = ( tau_lim_a - h_a);
}


} // namespace wolf_wbid

