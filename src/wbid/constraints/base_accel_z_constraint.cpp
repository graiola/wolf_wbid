#include <wolf_wbid/wbid/constraints/base_accel_z_constraint.h>
#include <stdexcept>

namespace wolf_wbid {

static inline double kBig() { return 1.0e20; }

BaseAccelZConstraint::BaseAccelZConstraint(const std::string& name,
                                           const IDVariables& vars,
                                           Mode mode,
                                           double eps)
: ConstraintBase(name)
, vars_(vars)
, cols_(vars.size())
, mode_(mode)
, eps_(eps)
{
  // We will build A (1 x nvars) and lA/uA (1)
  A_.setZero(1, cols_);
  lA_.setZero(1);
  uA_.setZero(1);

  const auto& qb = vars_.qddotBlock();
  if(qb.dim < 6)
    throw std::runtime_error("BaseAccelZConstraint: qddot block dim < 6 (no floating base?)");

  // base z is index 2 in the qddot block (assumption: [x y z roll pitch yaw ...])
  col_qddot_base_z_ = qb.offset + 2;

  if(col_qddot_base_z_ < 0 || col_qddot_base_z_ >= cols_)
    throw std::runtime_error("BaseAccelZConstraint: invalid column index for base z");
}

void BaseAccelZConstraint::update(const Eigen::VectorXd& /*x*/)
{
  // A * x within [lA, uA]
  A_.setZero();
  A_(0, col_qddot_base_z_) = 1.0;

  if(mode_ == Mode::EQUALITY)
  {
    lA_(0) = 0.0;
    uA_(0) = 0.0;
  }
  else // LOWER_BOUND
  {
    // qddot_base_z >= -eps
    lA_(0) = -eps_;
    uA_(0) =  kBig();
  }
}

} // namespace wolf_wbid

