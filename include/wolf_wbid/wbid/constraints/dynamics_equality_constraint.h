/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <wolf_wbid/wbid/constraints/constraint_base.h>
#include <wolf_wbid/wbid/id_variables.h>
#include <wolf_wbid/core/quadruped_robot.h>

namespace wolf_wbid {

/**
 * @brief Floating-base dynamics equality constraint (6 rows).
 *
 *   Bu(q) * qddot + hu(q,qd) - sum_i Jf_i(q)^T * f_i = 0
 *
 * Point-contact model only: `f_i = [Fx, Fy, Fz]`.
 * The implementation uses:
 *   Bu = B.topRows(6)
 *   hu = h.topRows(6)
 *   Jf = Jtmp.block<6,6>(0,0).transpose()
 */
class DynamicsEqualityConstraint final : public ConstraintBase
{
public:
  DynamicsEqualityConstraint(const std::string& name,
                            QuadrupedRobot& robot,
                            const IDVariables& vars,
                            const std::vector<std::string>& contact_links);

  void update(const Eigen::VectorXd& x) override;

  bool enableContact(const std::string& contact_link);
  bool disableContact(const std::string& contact_link);
  const std::vector<bool>& enabledContacts() const { return enabled_; }

private:
  int idxOf(const std::string& contact_link) const;

  QuadrupedRobot& robot_;
  const IDVariables& vars_;

  std::vector<std::string> contacts_;
  std::vector<bool> enabled_;

  // buffers
  Eigen::MatrixXd B_;     // n x n
  Eigen::VectorXd h_;     // n
  Eigen::MatrixXd Bu_;    // 6 x n
  Eigen::VectorXd hu_;    // 6
  Eigen::MatrixXd Jtmp_;  // 6 x n (robot dofs)
  Eigen::Matrix<double,6,6> Jf_; // 6 x 6 (base part transposed)

  // The base columns are assumed in the first 6 entries of qddot.
};

} // namespace wolf_wbid
