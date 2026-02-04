#pragma once
#ifndef WOLF_WBID_TASK_BASE_H
#define WOLF_WBID_TASK_BASE_H

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <cmath>   // std::isfinite
#include <utility> // std::move

#include <wolf_controller_utils/common.h>

namespace wolf_wbid {

/**
 * TaskBase
 * --------
 * Stable, minimal interface for all tasks used in the QP assembly.
 *
 * Each task produces a weighted least-squares term:
 *   minimize || diag(sqrt(wDiag)) * (A x - b) ||^2
 *
 * Where:
 *  - A: (m x n) matrix
 *  - b: (m) vector
 *  - wDiag: (m) non-negative row weights (NOT sqrt)
 *
 * A derived task fills A_, b_, w_diag_ (UNSCALED) in doUpdate().
 * After that, TaskBase applies scalar weight_ via finalizeWeights().
 */
class TaskBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit TaskBase(std::string task_id)
  : task_id_(std::move(task_id))
  {}

  virtual ~TaskBase() = default;

  // --- identity ---
  const std::string& id() const { return task_id_; }

  // --- dimensions ---
  int rows() const { return static_cast<int>(b_.size()); }
  int cols() const { return static_cast<int>(A_.cols()); }

  // --- LSQ term accessors (stable) ---
  const Eigen::MatrixXd& A() const { return A_; }
  const Eigen::VectorXd& b() const { return b_; }

  // NOTE: returned vector is AFTER finalizeWeights(), i.e., includes scalar weight_
  const Eigen::VectorXd& wDiag() const { return w_diag_; }

  // Convenience for debugging/ROS (allocates)
  Eigen::MatrixXd WdiagMatrix() const
  {
    if(w_diag_.size() == 0) return Eigen::MatrixXd();
    return w_diag_.asDiagonal();
  }

  // --- common knobs ---
  void setEnabled(bool en) { enabled_ = en; }
  bool enabled() const { return enabled_; }

  void setWeight(double w)
  {
    if(!(w >= 0.0)) throw std::runtime_error("TaskBase::setWeight: weight must be >= 0");
    weight_ = w;
  }
  double weight() const { return weight_; }

  void setLambda(double l1, double l2 = 0.0)
  {
    lambda1_ = l1;
    lambda2_ = l2;
  }
  double lambda1() const { return lambda1_; }
  double lambda2() const { return lambda2_; }

  void setGains(const Eigen::MatrixXd& Kp, const Eigen::MatrixXd& Kd)
  {
    Kp_ = Kp;
    Kd_ = Kd;
  }
  const Eigen::MatrixXd& getKp() const { return Kp_; }
  const Eigen::MatrixXd& getKd() const { return Kd_; }

  // --- update entrypoint (stable) ---
  void update(const Eigen::VectorXd& x)
  {
    if(!enabled_) {
      if(A_.size() > 0) A_.setZero();
      if(b_.size() > 0) b_.setZero();
      if(w_diag_.size() > 0) w_diag_.setZero();
      return;
    }
    doUpdate(x);
    finalizeWeights();
  }

  // --- reset entrypoint (legacy wrappers call CartesianTask::reset()) ---
  // NOTE: does NOT touch references; just clears term + re-enables.
  virtual void reset()
  {
    enabled_ = true;
    if(A_.size() > 0) A_.setZero();
    if(b_.size() > 0) b_.setZero();
    if(w_diag_.size() > 0) w_diag_.setZero();
  }

  // ============================================================================
  // Legacy-friendly aliases (to minimize wrapper churn)
  // ============================================================================

  // Historic name in wrappers: weightDiag was often treated as a scalar knob.
  // Keep the getter. DO NOT provide setWeightDiag here (ambiguity with wrapper).
  double weightDiag() const { return weight(); }

  // Historic wrappers also used setKp/setKd directly.
  void setKp(const Eigen::MatrixXd& Kp) { setGains(Kp, Kd_); }
  void setKd(const Eigen::MatrixXd& Kd) { setGains(Kp_, Kd); }

  // Some old code expected "getWeight()" to be a diagonal matrix.
  Eigen::MatrixXd getWeight() const { return WdiagMatrix(); }

  // Old “lambda” naming
  double getLambda()  const { return lambda1(); }
  double getLambda2() const { return lambda2(); }

  // If wrappers want to publish the row-weights vector explicitly
  const Eigen::VectorXd& getWeightDiag() const { return wDiag(); }

protected:
  // Derived tasks implement this and fill A_, b_, w_diag_ (unscaled)
  virtual void doUpdate(const Eigen::VectorXd& x) = 0;

  // Helpers for derived tasks:
  void resizeTerm(int m, int n)
  {
    A_.setZero(m, n);
    b_.setZero(m);
    w_diag_.setOnes(m);
  }

  // Apply scalar weight_ to w_diag_ and clamp numeric issues
  void finalizeWeights()
  {
    if(w_diag_.size() == 0) return;

    if(weight_ == 0.0) {
      w_diag_.setZero();
      return;
    }

    w_diag_ *= weight_;

    for(int i = 0; i < w_diag_.size(); ++i) {
      if(!(w_diag_(i) >= 0.0) || !std::isfinite(w_diag_(i))) w_diag_(i) = 0.0;
    }
  }

protected:
  std::string task_id_;
  bool enabled_{true};

  // Common knobs
  double weight_{1.0};
  double lambda1_{0.0};
  double lambda2_{0.0};

  // Generic gains
  Eigen::MatrixXd Kp_;
  Eigen::MatrixXd Kd_;

  // LSQ term
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  Eigen::VectorXd w_diag_;
};

} // namespace wolf_wbid
#endif // WOLF_WBID_TASK_BASE_H
