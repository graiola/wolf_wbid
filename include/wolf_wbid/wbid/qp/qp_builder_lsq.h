#pragma once
#include <vector>
#include <memory>
#include <Eigen/Dense>

#include <wolf_wbid/wbid/task_lsq.h>
#include <wolf_wbid/wbid/qp/qp_problem.h>

namespace wolf_wbid {

class IDVariables;

class QPBuilderLSQ
{
public:
  // Build only objective from LSQ tasks (plus optional small regularization)
  // H = 2 sum(A^T W A), g = -2 sum(A^T W b)
  static void buildObjectiveOnly(const IDVariables& vars,
                                const std::vector<ITaskLSQ*>& tasks,
                                QPProblem& qp_out,
                                double reg_eps = 0.0);
};

} // namespace wolf_wbid

