#include <wolf_wbid/wbid/constraints/contact_force_bounds_constraint.h>

#include <stdexcept>

namespace wolf_wbid {

ContactForceBoundsConstraint::ContactForceBoundsConstraint(const std::string& contact_name,
                                                           const IDVariables& vars,
                                                           const Eigen::Vector3d& f_min,
                                                           const Eigen::Vector3d& f_max)
: ConstraintBase(contact_name + "_force_bounds"),
  contact_(contact_name),
  vars_(vars),
  fmin_(f_min),
  fmax_(f_max)
{
  if(!vars_.hasContact(contact_))
    throw std::runtime_error("ContactForceBoundsConstraint: unknown contact " + contact_);

  if(vars_.contactDim() != 3)
    throw std::runtime_error("ContactForceBoundsConstraint: only POINT_CONTACT supported");

  resizeBounds(vars_.size());

  const auto& blk = vars_.contactBlock(contact_);
  l_.segment(blk.offset, 3) = fmin_;
  u_.segment(blk.offset, 3) = fmax_;
}

void ContactForceBoundsConstraint::setMin(const Eigen::Vector3d& f_min)
{
  fmin_ = f_min;
  const auto& blk = vars_.contactBlock(contact_);
  l_.segment(blk.offset, 3) = fmin_;
}

void ContactForceBoundsConstraint::setMax(const Eigen::Vector3d& f_max)
{
  fmax_ = f_max;
  const auto& blk = vars_.contactBlock(contact_);
  u_.segment(blk.offset, 3) = fmax_;
}

void ContactForceBoundsConstraint::update(const Eigen::VectorXd& /*x*/)
{
  // static bounds unless changed by setters
}

void ContactForceBoundsConstraint::releaseContact(bool release)
{
  const auto& blk = vars_.contactBlock(contact_);
  if(release)
  {
    l_.segment(blk.offset, 3).setZero();
    u_.segment(blk.offset, 3).setZero();
  }
  else
  {
    l_.segment(blk.offset, 3) = fmin_;
    u_.segment(blk.offset, 3) = fmax_;
  }
}

} // namespace wolf_wbid

