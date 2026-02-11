/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <wolf_wbid/wbid/constraints/constraint_base.h>
#include <wolf_wbid/wbid/id_variables.h>
#include <wolf_wbid/wbid/constraints/friction_cone_constraint.h>

namespace wolf_wbid {

/**
 * @brief Aggregates multiple FrictionConeConstraint objects into one linear constraint.
 *
 * Each cone contributes 5 rows. The class stacks all cones into a single
 * `(A, lA, uA)` constraint. It does not contribute variable bounds `(l, u)`.
 */
class FrictionConesConstraint final : public ConstraintBase
{
public:
  using Ptr = std::shared_ptr<FrictionConesConstraint>;

  FrictionConesConstraint(const std::string& name,
                          const std::vector<std::string>& contact_names,
                          const IDVariables& vars,
                          const std::vector<Eigen::Matrix3d>& wRl_list,
                          const std::vector<double>& mu_list);

  /** @brief Updates constraint data. */
  void update(const Eigen::VectorXd& x) override;

  FrictionConeConstraint::Ptr getFrictionCone(const std::string& contact_name);

  /** @brief Rebuilds stacked `(A, lA, uA)` from individual cones. */
  void rebuild();

private:
  std::vector<FrictionConeConstraint::Ptr> cones_;
};

} // namespace wolf_wbid
