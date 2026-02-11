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

  const int ndofs = robot_.getJointNum();
  if(tau_lim_.size() != ndofs)
    throw std::runtime_error("TorqueLimitsConstraint: torque_limits size != robot.getJointNum()");

  // Old behavior: constrain ALL dofs torques (base included).
  // If floating base, set tau_lim.head(6)=0 outside (as you already do in IDProblem::init()).
  resizeLinear(ndofs, vars_.size());
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

/*void TorqueLimitsConstraint::update(const Eigen::VectorXd&)
{
  robot_.getInertiaMatrix(B_);
  robot_.computeNonlinearTerm(h_);

  const int ndofs = robot_.getJointNum();
  if(B_.rows() != ndofs || B_.cols() != ndofs || h_.size() != ndofs)
    throw std::runtime_error("TorqueLimitsConstraint: inconsistent robot dynamics sizes");

  // IMPORTANT: full constraint on ALL dofs (incl floating base)
  resizeLinear(ndofs, vars_.size());
  A_.setZero();

  const auto& qb = vars_.qddotBlock();
  if(qb.dim != ndofs)
    throw std::runtime_error("TorqueLimitsConstraint: qddotBlock dim != ndofs");

  // A*x contains B*qddot - sum(J^T*w)
  A_.block(0, qb.offset, ndofs, ndofs) = B_;

  // POINT_CONTACT embedding: wrench6 = [f; 0]
  Eigen::Matrix<double,6,3> S_point;
  S_point.setZero();
  S_point.block<3,3>(0,0).setIdentity();

  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    if(!enabled_[i]) continue;

    const std::string& c = contacts_[i];
    if(!vars_.hasContact(c)) continue;

    robot_.getJacobian(c, Jtmp_); // 6 x ndofs
    if(Jtmp_.rows() < 6 || Jtmp_.cols() != ndofs)
      throw std::runtime_error("TorqueLimitsConstraint: bad Jacobian size for " + c);

    const auto& cb = vars_.contactBlock(c); // dim=3

    // tau += -J^T * wrench6 = -J^T * (S_point * f)
    A_.block(0, cb.offset, ndofs, 3).noalias() += -(Jtmp_.transpose() * S_point);
  }

  // bounds: -tau_lim - h <= A*x <= tau_lim - h
  // with tau_lim.head(6)=0 => base rows become equalities (old behavior)
  lA_ = (-tau_lim_ - h_);
  uA_ = ( tau_lim_ - h_);
}*/

void TorqueLimitsConstraint::update(const Eigen::VectorXd&)
{
  robot_.getInertiaMatrix(B_);
  robot_.computeNonlinearTerm(h_);

  const int ndofs = robot_.getJointNum();
  if(B_.rows() != ndofs || B_.cols() != ndofs || h_.size() != ndofs)
    throw std::runtime_error("TorqueLimitsConstraint: inconsistent robot dynamics sizes");

  // Constraint on all dofs (including floating base). Allocate once, then reuse.
  if(A_.rows() != ndofs || A_.cols() != vars_.size()) {
    resizeLinear(ndofs, vars_.size());
  } else {
    A_.setZero();
  }

  // qddot block deve essere ndofs (18 nel tuo caso)
  const auto& qb = vars_.qddotBlock();
  if(qb.dim != ndofs)
    throw std::runtime_error("TorqueLimitsConstraint: qddotBlock.dim != robot dofs");

  // A*x = B*qddot + sum(-J^T*wrench)
  A_.block(0, qb.offset, ndofs, ndofs) = B_;

  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    if(!enabled_[i]) continue;

    const std::string& c = contacts_[i];
    if(!vars_.hasContact(c)) continue;

    robot_.getJacobian(c, Jtmp_); // 6 x ndofs
    if(Jtmp_.rows() < 6 || Jtmp_.cols() != ndofs)
      throw std::runtime_error("TorqueLimitsConstraint: bad Jacobian size for " + c);

    const auto& cb = vars_.contactBlock(c);

    if(vars_.contactDim() == 3)
    {
      // POINT_CONTACT:
      // il tuo getJacobian() restituisce J = [J_lin; J_ang] (vedi swap in QuadrupedRobot)
      // quindi -J^T*[f;0] = -J_lin^T * f  ==> usa solo topRows(3)
      A_.block(0, cb.offset, ndofs, 3).noalias() += -Jtmp_.topRows(3).transpose();
    }
    else if(vars_.contactDim() == 6)
    {
      // SURFACE_CONTACT: -J^T * w
      A_.block(0, cb.offset, ndofs, 6).noalias() += -Jtmp_.transpose();
    }
    else
    {
      throw std::runtime_error("TorqueLimitsConstraint: unsupported contactDim()");
    }
  }

  // bounds: -tau_lim <= (B*qddot + sum(-J^T*w)) + h <= tau_lim
  // -> -tau_lim - h <= A*x <= tau_lim - h
  if(tau_lim_.size() != ndofs)
    throw std::runtime_error("TorqueLimitsConstraint: tau_lim size != ndofs");

  lA_ = (-tau_lim_ - h_);
  uA_ = ( tau_lim_ - h_);
}


} // namespace wolf_wbid
