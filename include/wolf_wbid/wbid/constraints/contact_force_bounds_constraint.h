#pragma once

#include <Eigen/Dense>
#include <string>

#include <wolf_wbid/wbid/constraints/constraint_base.h>
#include <wolf_wbid/wbid/id_variables.h>

namespace wolf_wbid {

/**
 * Adds bounds on the contact force variables for a given contact:
 *  l <= f <= u
 * Only POINT_CONTACT (3D) here.
 */
class ContactForceBoundsConstraint final : public ConstraintBase
{
public:
  ContactForceBoundsConstraint(const std::string& contact_name,
                               const IDVariables& vars,
                               const Eigen::Vector3d& f_min,
                               const Eigen::Vector3d& f_max);

  void update(const Eigen::VectorXd& x) override;

  void setMin(const Eigen::Vector3d& f_min);
  void setMax(const Eigen::Vector3d& f_max);

  void releaseContact(bool release);

private:
  std::string contact_;
  const IDVariables& vars_;
  Eigen::Vector3d fmin_{Eigen::Vector3d::Constant(-kInf)};
  Eigen::Vector3d fmax_{Eigen::Vector3d::Constant(+kInf)};
};

} // namespace wolf_wbid

