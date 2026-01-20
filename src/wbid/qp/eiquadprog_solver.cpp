#include <wolf_wbid/wbid/qp/eiquadprog_solver.h>

#include <limits>
#include <stdexcept>

// Your solver is in namespace Eigen and provides solve_quadprog(MatrixXd&, VectorXd&, ...)
#include <eigen_quadsolve.hpp>   // <-- put here the real header name you showed

namespace wolf_wbid {

EiQuadProgSolver::EiQuadProgSolver()
{
  // default same spirit as OpenSoT: eps_regularisation * BASE_REGULARISATION
  eps_reg_ = 1.0 * BASE_REGULARISATION;
}

void EiQuadProgSolver::setEpsRegularisation(double eps)
{
  if(eps < 0.0) throw std::runtime_error("EiQuadProgSolver: negative eps not allowed");
  eps_reg_ = eps * BASE_REGULARISATION;
}

static void appendCI(const Eigen::MatrixXd& CI_add, const Eigen::VectorXd& ci0_add,
                     Eigen::MatrixXd& CI, Eigen::VectorXd& ci0)
{
  // CI is (n x m), ci0 is (m)
  const int n = static_cast<int>(CI_add.rows());
  const int madd = static_cast<int>(CI_add.cols());
  const int mold = static_cast<int>(ci0.size());

  if(CI.size() == 0){
    CI = CI_add;
    ci0 = ci0_add;
    return;
  }

  if(CI.rows() != n) throw std::runtime_error("EiQuadProgSolver: CI row mismatch");

  Eigen::MatrixXd CI_new(n, mold + madd);
  Eigen::VectorXd ci0_new(mold + madd);

  CI_new.leftCols(mold) = CI;
  CI_new.rightCols(madd) = CI_add;

  ci0_new.head(mold) = ci0;
  ci0_new.tail(madd) = ci0_add;

  CI.swap(CI_new);
  ci0.swap(ci0_new);
}

QPSolution EiQuadProgSolver::solve(const QPProblem& qp)
{
  QPSolution sol;
  const int n = qp.n();
  if(n <= 0){
    sol.success = false;
    sol.status = "empty problem";
    return sol;
  }

  if(qp.H.rows() != n || qp.H.cols() != n || qp.g.size() != n){
    throw std::runtime_error("EiQuadProgSolver: invalid H/g sizes");
  }
  if(qp.A.rows() != qp.lA.size() || qp.A.rows() != qp.uA.size()){
    throw std::runtime_error("EiQuadProgSolver: A/lA/uA size mismatch");
  }
  if(qp.A.cols() != 0 && qp.A.cols() != n){
    throw std::runtime_error("EiQuadProgSolver: A cols mismatch");
  }

  // Copy because Eigen::solve_quadprog modifies G during Cholesky (per your header note)
  Eigen::MatrixXd H = qp.H;
  Eigen::VectorXd g = qp.g;

  // Regularization on diagonal (OpenSoT behavior)
  if(eps_reg_ > 0.0){
    H.diagonal().array() += eps_reg_;
  }

  // No equalities for now (same as OpenSoT backend you showed)
  Eigen::MatrixXd CE; CE.resize(n, 0);
  Eigen::VectorXd ce0; ce0.resize(0);

  // Build inequalities CI^T x + ci0 >= 0
  // We'll build CI as (n x m_ineq)
  Eigen::MatrixXd CI; CI.resize(n, 0);
  Eigen::VectorXd ci0; ci0.resize(0);

  // 1) Bounds l <= x <= u
  if(qp.l.size() == n && qp.u.size() == n){
    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);

    // I^T x + (-l) >= 0  -> x >= l
    appendCI(I, -qp.l, CI, ci0);

    // (-I)^T x + (u) >= 0 -> -x >= -u -> x <= u
    appendCI(-I, qp.u, CI, ci0);
  } else if(qp.l.size() != 0 || qp.u.size() != 0){
    throw std::runtime_error("EiQuadProgSolver: bounds must be either both size n or both empty");
  }

  // 2) Linear constraints lA <= A x <= uA
  if(qp.A.rows() > 0){
    // A x >= lA  -> (A^T)x + (-lA) >= 0
    appendCI(qp.A.transpose(), -qp.lA, CI, ci0);

    // A x <= uA  -> (-A)x >= -uA -> (-A^T)x + (uA) >= 0
    appendCI(-qp.A.transpose(), qp.uA, CI, ci0);
  }

  sol.x.setZero(n);

  const double inf = std::numeric_limits<double>::infinity();
  double obj = inf;

  // IMPORTANT: your signature is:
  //   Eigen::solve_quadprog(MatrixXd& G, VectorXd& g0, const MatrixXd& CE, const VectorXd& ce0,
  //                         const MatrixXd& CI, const VectorXd& ci0, VectorXd& x)
  obj = Eigen::solve_quadprog(H, g, CE, ce0, CI, ci0, sol.x);

  if(obj == inf || !std::isfinite(obj)){
    sol.success = false;
    sol.status = "infeasible/unbounded";
    sol.objective = obj;
    return sol;
  }

  sol.success = true;
  sol.status = "solved";
  sol.objective = obj;
  return sol;
}

} // namespace wolf_wbid

