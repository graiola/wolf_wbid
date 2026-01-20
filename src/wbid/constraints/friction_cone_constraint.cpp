#include <wolf_wbid/wbid/constraints/friction_cone_constraint.h>

#include <stdexcept>
#include <cmath>

namespace wolf_wbid {

FrictionConeConstraint::FrictionConeConstraint(const std::string& contact_name,
                                               const IDVariables& vars,
                                               double mu,
                                               const Eigen::Matrix3d& wRl)
: ConstraintBase(contact_name + "_friction_cone"),
  contact_(contact_name),
  vars_(vars),
  mu_(mu),
  wRl_(wRl)
{
  if(!vars_.hasContact(contact_))
    throw std::runtime_error("FrictionConeConstraint: unknown contact " + contact_);

  if(vars_.contactDim() != 3)
    throw std::runtime_error("FrictionConeConstraint: implemented only for POINT_CONTACT (dim=3)");

  resizeLinear(5, vars_.size());
  // inequality: A x <= 0  => lA = -inf, uA = 0
  lA_ = minusInfVec(5);
  uA_.setZero(5);

  computeCi();

  // Fill A only on contact block
  const auto& blk = vars_.contactBlock(contact_);
  A_.setZero();
  A_.block(0, blk.offset, 5, 3) = Ci_;
}

void FrictionConeConstraint::setMu(double mu)
{
  mu_ = mu;
  computeCi();
  const auto& blk = vars_.contactBlock(contact_);
  A_.setZero();
  A_.block(0, blk.offset, 5, 3) = Ci_;
}

void FrictionConeConstraint::setContactRotation(const Eigen::Matrix3d& wRl)
{
  wRl_ = wRl;
  computeCi();
  const auto& blk = vars_.contactBlock(contact_);
  A_.setZero();
  A_.block(0, blk.offset, 5, 3) = Ci_;
}

void FrictionConeConstraint::computeCi()
{
  // OpenSoT: mu/sqrt(2)
  const double mu_t = mu_ / std::sqrt(2.0);

  Eigen::Matrix<double,5,3> C;
  C.setZero();

  C(0,0) =  1.0; C(0,2) = -mu_t;
  C(1,0) = -1.0; C(1,2) = -mu_t;
  C(2,1) =  1.0; C(2,2) = -mu_t;
  C(3,1) = -1.0; C(3,2) = -mu_t;
  C(4,2) = -1.0;

  // same as OpenSoT: Ci = Ci * wRl^T
  Ci_ = C * wRl_.transpose();
}

void FrictionConeConstraint::update(const Eigen::VectorXd& /*x*/)
{
  // static unless mu / rotation changes at runtime
  // keep empty
}

} // namespace wolf_wbid
