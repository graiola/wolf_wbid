#pragma once
#include <wolf_wbid/wbid/qp/qp_solver.h>

#include <qpOASES.hpp>

#include <memory>

namespace wolf_wbid {

class QPOasesSolver final : public IQPSolver
{
public:
  QPOasesSolver();

  std::string name() const override { return "qpOASES"; }

  // Overrides qpOASES epsRegularisation (absolute value).
  void setEpsRegularisation(double eps) override;

  QPSolution solve(const QPProblem& qp) override;

  void setMaxWorkingSetRecalculations(int nWSR);

private:
  void ensureProblemSize(int n, int m);
  void applyOptions();

  // Matches OpenSoT convention (default multiplies qpOASES internal eps).
  static constexpr double BASE_REGULARISATION = 2.0e2;

  double eps_reg_{BASE_REGULARISATION};
  int nWSR_{13200};

  int last_n_{0};
  int last_m_{0};
  bool initialized_{false};

  std::unique_ptr<qpOASES::SQProblem> problem_;
  qpOASES::Options base_options_;
  qpOASES::Options options_;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_rm_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_rm_;
  Eigen::VectorXd g_;
  Eigen::VectorXd l_;
  Eigen::VectorXd u_;
  Eigen::VectorXd lA_;
  Eigen::VectorXd uA_;
  Eigen::VectorXd x_;
};

} // namespace wolf_wbid
