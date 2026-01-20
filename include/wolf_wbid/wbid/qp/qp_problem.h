#pragma once
#include <Eigen/Dense>
#include <string>

namespace wolf_wbid {

struct QPProblem
{
  // min 0.5 x^T H x + g^T x
  Eigen::MatrixXd H;
  Eigen::VectorXd g;

  // Linear constraints: lA <= A x <= uA
  Eigen::MatrixXd A;
  Eigen::VectorXd lA;
  Eigen::VectorXd uA;

  // Bounds: l <= x <= u
  Eigen::VectorXd l;
  Eigen::VectorXd u;

  int n() const { return static_cast<int>(g.size()); }
  int m() const { return static_cast<int>(A.rows()); }

  void resize(int nvars, int ncons = 0)
  {
    H.setZero(nvars, nvars);
    g.setZero(nvars);

    A.resize(ncons, nvars);
    lA.resize(ncons);
    uA.resize(ncons);

    l.resize(0);
    u.resize(0);
  }
};

struct QPSolution
{
  bool success{false};
  double objective{0.0};
  Eigen::VectorXd x;
  std::string status;
};

} // namespace wolf_wbid
