#include <wolf_wbid/wbid/tasks/task_base.h>

namespace wolf_wbid {

TaskBase::TaskBase(std::string id)
: id_(std::move(id))
{
  if(id_.empty())
    throw std::runtime_error("TaskBase: empty id");
}

void TaskBase::resize(int rows, int cols)
{
  if(rows < 0 || cols < 0)
    throw std::runtime_error("TaskBase::resize(): negative sizes");

  A_.setZero(rows, cols);
  b_.setZero(rows);

  setDefaultRowWeights(rows);
  finalizeWeights();
}

void TaskBase::setDefaultRowWeights(int rows)
{
  w_diag_user_  = Eigen::VectorXd::Ones(rows);
  w_diag_final_ = Eigen::VectorXd::Ones(rows);
  W_.setIdentity(rows, rows);
}

void TaskBase::finalizeWeights()
{
  const int m = static_cast<int>(w_diag_user_.size());

  if(!std::isfinite(weight_scalar_) || weight_scalar_ < 0.0)
    throw std::runtime_error("TaskBase: weight_scalar is not finite or negative");

  w_diag_final_.resize(m);
  for(int i = 0; i < m; ++i) {
    const double u = w_diag_user_(i);
    if(!std::isfinite(u) || u < 0.0)
      throw std::runtime_error("TaskBase: user weights must be finite and >= 0");
    const double v = weight_scalar_ * u;
    w_diag_final_(i) = (std::isfinite(v) && v >= 0.0) ? v : 0.0;
  }

  W_.setZero(m, m);
  if(m > 0) W_.diagonal() = w_diag_final_;
}

void TaskBase::setLambda(double l1, double l2)
{
  if(!isFinite(l1) || !isFinite(l2))
    throw std::runtime_error("TaskBase::setLambda(): non-finite lambda");
  if(l1 < 0.0 || l2 < 0.0)
    throw std::runtime_error("TaskBase::setLambda(): negative lambda not allowed");

  lambda1_ = l1;
  lambda2_ = l2;
}

void TaskBase::setWeightScalar(double w)
{
  if(!isFinite(w) || w < 0.0)
    throw std::runtime_error("TaskBase::setWeightScalar(): weight must be finite and >= 0");

  weight_scalar_ = w;
  finalizeWeights();
}

void TaskBase::setWeightDiag(const Eigen::VectorXd& w_diag)
{
  if(w_diag.size() != rows())
    throw std::runtime_error("TaskBase::setWeightDiag(): size mismatch");

  w_diag_user_ = w_diag; // validated in finalizeWeights()
  finalizeWeights();
}

void TaskBase::setWeightDiag(double w)
{
  if(!isFinite(w) || w < 0.0)
    throw std::runtime_error("TaskBase::setWeightDiag(scalar): weight must be finite and >= 0");

  w_diag_user_ = Eigen::VectorXd::Ones(rows()); // constant diag handled by scalar multiplier
  weight_scalar_ = w;
  finalizeWeights();
}

void TaskBase::setWeight(const Eigen::MatrixXd& Wdiag)
{
  if(Wdiag.rows() != rows() || Wdiag.cols() != rows())
    throw std::runtime_error("TaskBase::setWeight(W): W must be square and match task rows");

  // We only use diagonal entries as per-row weights (OpenSoT-like).
  Eigen::VectorXd w(rows());
  for(int i = 0; i < rows(); ++i) {
    const double v = Wdiag(i,i);
    if(!std::isfinite(v) || v < 0.0)
      throw std::runtime_error("TaskBase::setWeight(W): diag entries must be finite and >= 0");
    w(i) = v;
  }
  setWeightDiag(w);
}

void TaskBase::setKp(const Eigen::MatrixXd& Kp)
{
  if(Kp.size() == 0) { Kp_.resize(0,0); return; }
  if(Kp.rows() != Kp.cols())
    throw std::runtime_error("TaskBase::setKp(): Kp must be square");

  for(int i = 0; i < Kp.rows(); ++i)
    for(int j = 0; j < Kp.cols(); ++j)
      if(!std::isfinite(Kp(i,j)))
        throw std::runtime_error("TaskBase::setKp(): non-finite value");

  Kp_ = Kp;
}

void TaskBase::setKd(const Eigen::MatrixXd& Kd)
{
  if(Kd.size() == 0) { Kd_.resize(0,0); return; }
  if(Kd.rows() != Kd.cols())
    throw std::runtime_error("TaskBase::setKd(): Kd must be square");

  for(int i = 0; i < Kd.rows(); ++i)
    for(int j = 0; j < Kd.cols(); ++j)
      if(!std::isfinite(Kd(i,j)))
        throw std::runtime_error("TaskBase::setKd(): non-finite value");

  Kd_ = Kd;
}

} // namespace wolf_wbid
