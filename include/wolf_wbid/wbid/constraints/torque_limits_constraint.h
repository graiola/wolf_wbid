// wolf_wbid/wbid/constraints/torque_limits_constraint.h
#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <wolf_wbid/wbid/constraints/constraint_base.h>
#include <wolf_wbid/wbid/id_variables.h>
#include <wolf_wbid/core/quadruped_robot.h>

namespace wolf_wbid {

/**
 * Torque limits inequality:
 *
 * tau = B(q) * qddot + h(q,qd) - sum_i J_i(q)^T * wrench_i
 *
 * Enforce:
 *   -tau_lim <= tau <= tau_lim
 *
 * Rewritten as linear constraint in x:
 *   lA <= A x <= uA
 *
 * where:
 *   A * x = B*qddot - sum_i J^T * w_i
 *   lA = -tau_lim - h
 *   uA =  tau_lim - h
 */
class TorqueLimitsConstraint final : public ConstraintBase
{
public:
  TorqueLimitsConstraint(const std::string& name,
                         QuadrupedRobot& robot,
                         const IDVariables& vars,
                         const std::vector<std::string>& contact_links,
                         const Eigen::VectorXd& torque_limits);

  void update(const Eigen::VectorXd& x) override;

  bool enableContact(const std::string& contact_link);
  bool disableContact(const std::string& contact_link);
  const std::vector<bool>& enabledContacts() const { return enabled_; }

  void setTorqueLimits(const Eigen::VectorXd& tau_lim);

private:
  int idxOf(const std::string& contact_link) const;

  QuadrupedRobot& robot_;
  const IDVariables& vars_;

  std::vector<std::string> contacts_;
  std::vector<bool> enabled_;

  Eigen::VectorXd tau_lim_;

  // buffers
  Eigen::MatrixXd B_;
  Eigen::VectorXd h_;
  Eigen::MatrixXd Jtmp_; // 6 x ndofs
};

} // namespace wolf_wbid

