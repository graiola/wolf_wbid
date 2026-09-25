#include <wolf_wbid/wbid/qp/eiquadprog_solver.h>

#include <limits>
#include <stdexcept>
#include <algorithm>
#include <cmath>

// Your solver is in namespace Eigen and provides solve_quadprog(MatrixXd&, VectorXd&, ...)
#include <eiquadprog.hpp>   // <-- put here the real header name you showed

namespace wolf_wbid {

EiQuadProgSolver::EiQuadProgSolver()
{
  // Default: same spirit as legacy backend
  // (base numerical regularisation)
  eps_reg_ = 1.0 * BASE_REGULARISATION;
}

void EiQuadProgSolver::setEpsRegularisation(double eps)
{
  if(eps < 0.0) {
    throw std::runtime_error("EiQuadProgSolver: negative eps not allowed");
  }

  // Keep the same "multiplier * BASE_REGULARISATION" convention,
  // but avoid going below BASE_REGULARISATION if eps > 0.
  // eps == 0 -> disable (numerically risky, but explicit).
  if(eps == 0.0) {
    eps_reg_ = 0.0;
  } else {
    eps_reg_ = std::max(BASE_REGULARISATION, eps * BASE_REGULARISATION);
  }
}

static void appendCol(Eigen::MatrixXd& M, Eigen::VectorXd& v, const Eigen::VectorXd& col, double val)
{
  const int n = static_cast<int>(col.size());
  const int mold = static_cast<int>(v.size());
  M.conservativeResize(n, mold + 1);
  M.col(mold) = col;
  v.conservativeResize(mold + 1);
  v(mold) = val;
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

  // Copy because Eigen::solve_quadprog modifies G during Cholesky
  Eigen::MatrixXd H = qp.H;
  Eigen::VectorXd g = qp.g;

  // Numerical diagonal regularisation (backend stability)
  if(eps_reg_ > 0.0){
    H.diagonal().array() += eps_reg_;
  }

  constexpr double kUnbounded = 1.0e19;
  constexpr double kEqTol = 1.0e-8;

  Eigen::MatrixXd CE(n, 0);
  Eigen::VectorXd ce0(0);

  Eigen::MatrixXd CI(n, 0);
  Eigen::VectorXd ci0(0);

  // 1) Bounds l <= x <= u
  if(qp.l.size() == n && qp.u.size() == n){
    for(int i = 0; i < n; ++i){
      const double li = qp.l(i);
      const double ui = qp.u(i);

      if(li > -kUnbounded && ui < kUnbounded && std::abs(ui - li) < kEqTol){
        // Equality x(i) = li => e_i^T x - li = 0
        Eigen::VectorXd e = Eigen::VectorXd::Zero(n);
        e(i) = 1.0;
        appendCol(CE, ce0, e, -li);
      } else {
        if(li > -kUnbounded){
          // x(i) >= li => e_i^T x - li >= 0
          Eigen::VectorXd e = Eigen::VectorXd::Zero(n);
          e(i) = 1.0;
          appendCol(CI, ci0, e, -li);
        }
        if(ui < kUnbounded){
          // x(i) <= ui => -e_i^T x + ui >= 0
          Eigen::VectorXd e = Eigen::VectorXd::Zero(n);
          e(i) = -1.0;
          appendCol(CI, ci0, e, ui);
        }
      }
    }
  } else if(qp.l.size() != 0 || qp.u.size() != 0){
    throw std::runtime_error("EiQuadProgSolver: bounds must be either both size n or both empty");
  }

  // 2) Linear constraints lA <= A x <= uA
  if(qp.A.rows() > 0){
    for(int i = 0; i < qp.A.rows(); ++i){
      const double lAi = qp.lA(i);
      const double uAi = qp.uA(i);

      if(lAi > -kUnbounded && uAi < kUnbounded && std::abs(uAi - lAi) < kEqTol){
        // Equality A.row(i) x - lAi = 0
        appendCol(CE, ce0, qp.A.row(i).transpose(), -lAi);
      } else {
        if(lAi > -kUnbounded){
          // A.row(i) x >= lAi => A.row(i) x - lAi >= 0
          appendCol(CI, ci0, qp.A.row(i).transpose(), -lAi);
        }
        if(uAi < kUnbounded){
          // A.row(i) x <= uAi => -A.row(i) x + uAi >= 0
          appendCol(CI, ci0, -qp.A.row(i).transpose(), uAi);
        }
      }
    }
  }

  sol.x.setZero(n);

  const double inf = std::numeric_limits<double>::infinity();
  double obj = inf;

  // Signature:
  //   Eigen::solve_quadprog(MatrixXd& G, VectorXd& g0,
  //                         const MatrixXd& CE, const VectorXd& ce0,
  //                         const MatrixXd& CI, const VectorXd& ci0,
  //                         VectorXd& x)
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
