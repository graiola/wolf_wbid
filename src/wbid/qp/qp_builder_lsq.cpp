#include <wolf_wbid/wbid/qp/qp_builder_lsq.h>
#include <wolf_wbid/wbid/id_variables.h>

namespace wolf_wbid {

void QPBuilderLSQ::buildObjectiveOnly(const IDVariables& vars,
                                      const std::vector<ITaskLSQ*>& tasks,
                                      QPProblem& qp_out,
                                      double reg_eps)
{
  const int n = vars.size();
  qp_out.resize(n);

  LsqTerm term;
  for(const auto* t : tasks){
    if(!t) continue;
    t->compute(vars, term);

    // W is diagonal: w_diag (m)
    // Accumulate: H += 2 A^T W A, g += -2 A^T W b
    // Implement as: scale rows of A and b by sqrt(w) then do normal equations
    const int m = static_cast<int>(term.b.size());
    if(m == 0) continue;

    Eigen::VectorXd sqrtw = term.w_diag.cwiseMax(0.0).cwiseSqrt();
    Eigen::MatrixXd Aw = term.A;
    Eigen::VectorXd bw = term.b;

    // row scaling
    Aw = sqrtw.asDiagonal() * Aw;
    bw = sqrtw.asDiagonal() * bw;

    qp_out.H.noalias() += 2.0 * (Aw.transpose() * Aw);
    qp_out.g.noalias() += -2.0 * (Aw.transpose() * bw);
  }

  if(reg_eps > 0.0){
    qp_out.H.diagonal().array() += reg_eps;
  }
}

} // namespace wolf_wbid

