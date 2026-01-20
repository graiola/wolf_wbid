#pragma once
#include <Eigen/Dense>
#include <string>

namespace wolf_wbid {

class IDVariables;

struct LsqTerm
{
  Eigen::MatrixXd A;     // (m x n)
  Eigen::VectorXd b;     // (m)
  Eigen::VectorXd w_diag;// (m) non-negative
};

class ITaskLSQ
{
public:
  virtual ~ITaskLSQ() = default;
  virtual std::string name() const = 0;

  // produce A,b,w for current model/refs (using vars to know n and block offsets)
  virtual void compute(const IDVariables& vars, LsqTerm& out) = 0;
};

} // namespace wolf_wbid
