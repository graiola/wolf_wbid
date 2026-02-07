#include <wolf_wbid/wbid/qp/qpoases_solver.h>

#include <limits>
#include <stdexcept>
#include <algorithm>

namespace wolf_wbid {

namespace {

inline std::string statusToString(qpOASES::returnValue val)
{
  return std::string(qpOASES::MessageHandling::getErrorCodeMessage(val));
}

inline bool isSuccess(qpOASES::returnValue val)
{
  return qpOASES::getSimpleStatus(val) >= 0;
}

} // namespace

QPOasesSolver::QPOasesSolver()
{
  base_options_.setToMPC();
  base_options_.printLevel = qpOASES::PL_NONE;
  base_options_.enableRegularisation = qpOASES::BT_FALSE;
  base_options_.numRegularisationSteps = 0;
  base_options_.numRefinementSteps = 1;

  options_ = base_options_;
  options_.epsRegularisation *= BASE_REGULARISATION;
  eps_reg_ = options_.epsRegularisation;

  applyOptions();
}

void QPOasesSolver::setMaxWorkingSetRecalculations(int nWSR)
{
  if(nWSR <= 0) {
    throw std::runtime_error("QPOasesSolver: nWSR must be > 0");
  }
  nWSR_ = nWSR;
}

void QPOasesSolver::setEpsRegularisation(double eps)
{
  if(eps < 0.0) {
    throw std::runtime_error("QPOasesSolver: negative eps not allowed");
  }

  eps_reg_ = eps;
  applyOptions();
}

void QPOasesSolver::applyOptions()
{
  options_ = base_options_;
  options_.epsRegularisation = eps_reg_;
  options_.ensureConsistency();

  if(problem_) {
    problem_->setOptions(options_);
  }
}

void QPOasesSolver::ensureProblemSize(int n, int m)
{
  if(n <= 0) throw std::runtime_error("QPOasesSolver: empty problem");

  if(n != last_n_ || m != last_m_ || !problem_) {
    problem_ = std::make_unique<qpOASES::SQProblem>(n, m, qpOASES::HST_UNKNOWN);
    problem_->setOptions(options_);
    last_n_ = n;
    last_m_ = m;
    initialized_ = false;
  }
}

QPSolution QPOasesSolver::solve(const QPProblem& qp)
{
  QPSolution sol;
  const int n = qp.n();
  const int m = qp.m();

  if(n <= 0) {
    sol.success = false;
    sol.status = "empty problem";
    return sol;
  }

  if(qp.H.rows() != n || qp.H.cols() != n || qp.g.size() != n) {
    throw std::runtime_error("QPOasesSolver: invalid H/g sizes");
  }
  if(qp.A.cols() != 0 && qp.A.cols() != n) {
    throw std::runtime_error("QPOasesSolver: A cols mismatch");
  }
  if(qp.A.rows() != qp.lA.size() || qp.A.rows() != qp.uA.size()) {
    throw std::runtime_error("QPOasesSolver: A/lA/uA size mismatch");
  }

  ensureProblemSize(n, m);

  H_rm_ = qp.H;
  if(eps_reg_ > 0.0) {
    H_rm_.diagonal().array() += eps_reg_;
  }
  g_ = qp.g;

  if(m > 0) {
    A_rm_ = qp.A;
    lA_ = qp.lA;
    uA_ = qp.uA;
  } else {
    A_rm_.resize(0, n);
    lA_.resize(0);
    uA_.resize(0);
  }

  const double inf = qpOASES::INFTY;
  const auto clampBounds = [inf](Eigen::VectorXd& l, Eigen::VectorXd& u) {
    for(int i = 0; i < l.size(); ++i) {
      if(l[i] < -inf) l[i] = -inf;
      if(u[i] >  inf) u[i] =  inf;
    }
  };

  if(qp.l.size() == n && qp.u.size() == n) {
    l_ = qp.l;
    u_ = qp.u;
  } else if(qp.l.size() == 0 && qp.u.size() == 0) {
    l_ = Eigen::VectorXd::Constant(n, -inf);
    u_ = Eigen::VectorXd::Constant(n,  inf);
  } else {
    throw std::runtime_error("QPOasesSolver: bounds must be either both size n or both empty");
  }
  clampBounds(l_, u_);

  if(m > 0) {
    clampBounds(lA_, uA_);
  }

  int nWSR = nWSR_;
  qpOASES::returnValue rv;

  const double* A_ptr = (m > 0) ? A_rm_.data() : nullptr;
  const double* lA_ptr = (m > 0) ? lA_.data() : nullptr;
  const double* uA_ptr = (m > 0) ? uA_.data() : nullptr;

  if(!initialized_) {
    rv = problem_->init(H_rm_.data(), g_.data(), A_ptr, l_.data(), u_.data(), lA_ptr, uA_ptr, nWSR);
  } else {
    rv = problem_->hotstart(H_rm_.data(), g_.data(), A_ptr, l_.data(), u_.data(), lA_ptr, uA_ptr, nWSR);
    if(!isSuccess(rv)) {
      problem_ = std::make_unique<qpOASES::SQProblem>(n, m, qpOASES::HST_UNKNOWN);
      problem_->setOptions(options_);
      initialized_ = false;
      nWSR = nWSR_;
      rv = problem_->init(H_rm_.data(), g_.data(), A_ptr, l_.data(), u_.data(), lA_ptr, uA_ptr, nWSR);
    }
  }

  if(!isSuccess(rv)) {
    sol.success = false;
    sol.status = statusToString(rv);
    sol.objective = std::numeric_limits<double>::infinity();
    return sol;
  }

  initialized_ = true;

  x_.resize(n);
  qpOASES::returnValue rv_sol = problem_->getPrimalSolution(x_.data());
  if(rv_sol != qpOASES::SUCCESSFUL_RETURN) {
    sol.success = false;
    sol.status = statusToString(rv_sol);
    sol.objective = std::numeric_limits<double>::infinity();
    return sol;
  }

  sol.success = true;
  sol.status = statusToString(rv);
  sol.objective = problem_->getObjVal();
  sol.x = x_;
  return sol;
}

} // namespace wolf_wbid
