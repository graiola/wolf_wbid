#pragma once
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <wolf_wbid/wbid/qp/qp_problem.h>
#include <wolf_wbid/wbid/qp/qp_solver.h>
#include <wolf_wbid/wbid/task_lsq.h>

namespace wolf_wbid {

class IDVariables;

/**
 * A "level" contains LSQ tasks that will be summed together.
 * For now: all levels are merged into a single QP (soft priorities).
 * Future: implement lexicographic HQP if needed.
 */
struct QPStackLevel
{
  std::string name;
  std::vector<ITaskLSQ*> tasks; // non-owning
};

class QPStack
{
public:
  using Ptr = std::shared_ptr<QPStack>;

  explicit QPStack(std::string name = "qp_stack");

  // --- Task structure ---
  void clearLevels();
  void addLevel(const std::string& level_name, const std::vector<ITaskLSQ*>& tasks);

  // Convenience: single unnamed level
  void setSingleLevel(const std::vector<ITaskLSQ*>& tasks);

  // --- Problem constraints ---
  // Bounds: l <= x <= u
  void setBounds(const Eigen::VectorXd& l, const Eigen::VectorXd& u);
  void clearBounds();

  // Linear constraints: lA <= A x <= uA
  void setLinearConstraints(const Eigen::MatrixXd& A,
                            const Eigen::VectorXd& lA,
                            const Eigen::VectorXd& uA);
  void clearLinearConstraints();

  // Regularization on H diagonal (in addition to solver eps)
  void setDiagonalRegularization(double reg_eps);

  // Build problem for current vars (does not solve)
  void update(const IDVariables& vars);

  // Solve last built problem
  QPSolution solve(IQPSolver& solver);

  // Accessors
  const QPProblem& problem() const { return qp_; }
  QPProblem& problem() { return qp_; }

private:
  std::string name_;
  std::vector<QPStackLevel> levels_;
  QPProblem qp_;
  double reg_eps_{0.0};

  // cached bounds/constraints (so you can set once, update many times)
  bool has_bounds_{false};
  Eigen::VectorXd l_, u_;

  bool has_lin_{false};
  Eigen::MatrixXd A_;
  Eigen::VectorXd lA_, uA_;
};

} // namespace wolf_wbid

