#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <wolf_wbid/wbid/constraint_base.h>
#include <wolf_wbid/wbid/id_variables.h>
#include <wolf_wbid/wbid/constraints/friction_cone_constraint.h>

namespace wolf_wbid {

class FrictionConesConstraint : public IConstraint
{
public:
  using Ptr = std::shared_ptr<FrictionConesConstraint>;

  FrictionConesConstraint(const std::string& name,
                          const std::vector<std::string>& contact_names,
                          const IDVariables& vars,
                          const std::vector<Eigen::Matrix3d>& wRl_list,
                          const std::vector<double>& mu_list);

  void update(const Eigen::VectorXd& x) override; // no-op
  const Eigen::MatrixXd& Aineq() const override { return Aineq_; }
  const Eigen::VectorXd& bLowerBound() const override { return bLower_; }
  const Eigen::VectorXd& bUpperBound() const override { return bUpper_; }
  int rows() const override { return static_cast<int>(bUpper_.size()); }
  const std::string& name() const override { return name_; }

  FrictionConeConstraint::Ptr getFrictionCone(const std::string& contact_name);

  void rebuild(); // re-stack internal cones into Aineq/bounds

private:
  std::string name_;
  std::vector<FrictionConeConstraint::Ptr> cones_;

  Eigen::MatrixXd Aineq_;
  Eigen::VectorXd bLower_;
  Eigen::VectorXd bUpper_;
};

} // namespace wolf_wbid

