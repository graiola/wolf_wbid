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
 * Aggregatore: impila N FrictionConeConstraint (5 righe ciascuna)
 * in un'unica constraint lineare (A, lA, uA) conforme a IConstraint.
 *
 * Nota: non aggiunge bounds su variabili (l/u), solo vincoli lineari.
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

  void update(const Eigen::VectorXd& x) override; // di default no-op (o rebuild se vuoi)

  FrictionConeConstraint::Ptr getFrictionCone(const std::string& contact_name);

  // Re-impila A/lA/uA dai singoli coni
  void rebuild();

private:
  std::vector<FrictionConeConstraint::Ptr> cones_;
};

} // namespace wolf_wbid
