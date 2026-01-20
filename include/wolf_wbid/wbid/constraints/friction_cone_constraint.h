#pragma once

#include <Eigen/Dense>
#include <string>

#include <wolf_wbid/wbid/constraint_base.h>
#include <wolf_wbid/wbid/id_variables.h>

namespace wolf_wbid {

/**
 * Point-contact friction pyramid (5 inequalities):
 *  Fx - mu/sqrt(2)*Fz <= 0
 * -Fx - mu/sqrt(2)*Fz <= 0
 *  Fy - mu/sqrt(2)*Fz <= 0
 * -Fy - mu/sqrt(2)*Fz <= 0
 *           -Fz <= 0   (i.e., Fz >= 0)
 *
 * Then rotated by wRl (same OpenSoT behavior): Ci = Ci * wRl^T
 */
class FrictionConeConstraint final : public ConstraintBase
{
public:
  FrictionConeConstraint(const std::string& contact_name,
                         const IDVariables& vars,
                         double mu,
                         const Eigen::Matrix3d& wRl);

  void update(const Eigen::VectorXd& x) override;

  void setMu(double mu);
  void setContactRotation(const Eigen::Matrix3d& wRl);

private:
  void computeCi();

  std::string contact_;
  const IDVariables& vars_;

  double mu_{1.0};
  Eigen::Matrix3d wRl_{Eigen::Matrix3d::Identity()};

  Eigen::Matrix<double,5,3> Ci_{Eigen::Matrix<double,5,3>::Zero()};
};

} // namespace wolf_wbid
