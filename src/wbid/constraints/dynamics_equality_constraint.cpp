#include <wolf_wbid/wbid/constraints/dynamics_equality_constraint.h>

#include <algorithm>
#include <stdexcept>

namespace wolf_wbid {

static inline void setEqRows(Eigen::VectorXd& lA, Eigen::VectorXd& uA, const Eigen::VectorXd& b)
{
  lA = b;
  uA = b;
}

DynamicsEqualityConstraint::DynamicsEqualityConstraint(const std::string& name,
                                                       QuadrupedRobot& robot,
                                                       const IDVariables& vars,
                                                       const std::vector<std::string>& contact_links)
: ConstraintBase(name),
  robot_(robot),
  vars_(vars),
  contacts_(contact_links)
{
  if(vars_.contactDim() != 3)
    throw std::runtime_error("DynamicsEqualityConstraint: implemented only for POINT_CONTACT (dim=3)");

  enabled_.assign(contacts_.size(), true);

  // 6 equality rows, n vars columns
  resizeLinear(6, vars_.size());

  // We'll set lA=uA every update
  lA_.setZero(6);
  uA_.setZero(6);
}

int DynamicsEqualityConstraint::idxOf(const std::string& contact_link) const
{
  auto it = std::find(contacts_.begin(), contacts_.end(), contact_link);
  if(it == contacts_.end()) return -1;
  return static_cast<int>(std::distance(contacts_.begin(), it));
}

bool DynamicsEqualityConstraint::enableContact(const std::string& contact_link)
{
  int i = idxOf(contact_link);
  if(i < 0) return false;
  enabled_[static_cast<size_t>(i)] = true;
  return true;
}

bool DynamicsEqualityConstraint::disableContact(const std::string& contact_link)
{
  int i = idxOf(contact_link);
  if(i < 0) return false;
  enabled_[static_cast<size_t>(i)] = false;
  return true;
}

void DynamicsEqualityConstraint::update(const Eigen::VectorXd& /*x*/)
{
  // 1) get B(q), h(q,qd)
  robot_.getInertiaMatrix(B_);
  robot_.computeNonlinearTerm(h_);

  if(B_.rows() < 6 || B_.cols() < 6 || h_.size() < 6)
    throw std::runtime_error("DynamicsEqualityConstraint: robot model seems not floating-base (missing 6 dofs)");

  Bu_ = B_.topRows(6);
  hu_ = h_.head(6);

  // 2) Build equality: Bu*qddot - sum(Jf*f) = -hu
  // Our QP variable x contains qddot block (full robot dofs, incl base first 6) then contact forces.
  // We put everything into A_ * x = b, with b = -hu.
  A_.setZero();

  // qddot block
  const auto& qb = vars_.qddotBlock();
  A_.block(0, qb.offset, 6, qb.dim) = Bu_;

  // 3) contact contributions: (-Jf)*f
  // OpenSoT uses Jtmp.block<6,6>(0,0).transpose(), i.e. base columns only.
  // So we multiply that by contact force (3D) assuming point contact produces base wrench via those 6 cols transpose?:
  // In OpenSoT, wrenches[i] is 6D or 3D depending on contact model; but in your point contact setup it's 3D.
  // Their code still does (-Jf) * wrench[i]; that implies Jf is 6x3 for point contact in their serializer.
  //
  // Here we replicate the intent: map force (3) to base wrench using contact Jacobian at the contact point.
  // We can obtain the full spatial Jacobian Jtmp (6 x n) of the contact link expressed in world.
  // The generalized force from a point force is Jv_base^T * f, where Jv are the linear rows.
  //
  // BUT OpenSoT uses the 6x6 block (base part) transpose and multiplies by wrench variable.
  // For POINT_CONTACT, safest consistent mapping:
  //   use only linear part of base Jacobian: Jlin_base (3x6), then contribution to base eq is -Jlin_base^T * f (6x1)
  // This yields 6 rows from 3 force components, as physically correct.
  //
  // So we implement: A_contact_block = -Jlin_base^T, where Jlin_base = Jtmp.topRows(3).leftCols(6).
  //
  // This matches the “floating base dynamics with external forces” and avoids an inconsistent 6x6*3 multiply.

  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    if(!enabled_[i]) continue;

    const std::string& c = contacts_[i];
    if(!vars_.hasContact(c)) continue; // ignore if not in variable layout

    // get Jacobian of contact link in world
    robot_.getJacobian(c, Jtmp_); // expected 6 x n

    if(Jtmp_.rows() < 3 || Jtmp_.cols() < 6)
      throw std::runtime_error("DynamicsEqualityConstraint: Jacobian too small for contact " + c);

    Eigen::Matrix<double,3,6> Jlin_base = Jtmp_.block<3,6>(0,0); // linear rows, base cols

    const auto& cb = vars_.contactBlock(c);
    // A_ rows are 6, contact var dim is 3
    // contribution: -Jlin_base^T * f
    A_.block(0, cb.offset, 6, 3).noalias() += -Jlin_base.transpose();
  }

  // b = -hu
  Eigen::VectorXd b = -hu_;
  setEqRows(lA_, uA_, b);
}

} // namespace wolf_wbid

