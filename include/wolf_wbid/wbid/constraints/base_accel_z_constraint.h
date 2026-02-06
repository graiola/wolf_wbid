#pragma once

#include <wolf_wbid/wbid/constraints/constraint_base.h>
#include <wolf_wbid/wbid/id_variables.h>

namespace wolf_wbid {

/**
 * Linear constraint on floating-base vertical acceleration:
 *   Option A (equality):   qddot_base_z = 0
 *   Option B (inequality): qddot_base_z >= -eps
 *
 * Assumes qddot block starts with 6 floating-base dofs and z is index 2.
 */
class BaseAccelZConstraint final : public ConstraintBase
{
public:
  enum class Mode { EQUALITY, LOWER_BOUND };

  BaseAccelZConstraint(const std::string& name,
                       const IDVariables& vars,
                       Mode mode,
                       double eps = 0.0);

  int rows() const override { return 1; }
  int cols() const override { return cols_; }

  void update(const Eigen::VectorXd& x) override;

  void setMode(Mode m) { mode_ = m; }
  void setEps(double eps) { eps_ = eps; }   // used only for LOWER_BOUND

private:
  const IDVariables& vars_;
  int cols_{0};

  Mode mode_{Mode::EQUALITY};
  double eps_{0.0};

  int col_qddot_base_z_{-1}; // absolute column index in x
};

} // namespace wolf_wbid
