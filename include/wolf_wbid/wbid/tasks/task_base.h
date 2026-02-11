/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#pragma once
#ifndef WOLF_WBID_TASK_BASE_H
#define WOLF_WBID_TASK_BASE_H

#include <Eigen/Dense>
#include <string>
#include <stdexcept>
#include <cmath>

#include <wolf_controller_utils/common.h>

namespace wolf_wbid {

/**
 * @brief Common base for LSQ tasks.
 *
 * Residual: r(x) = A x - b
 * Weighting: 0.5 * r' W r, with W diagonal (row-weights).
 *
 * The class stores:
 *  - w_diag_user_   : user-provided per-row weights (>=0)
 *  - weight_scalar_ : scalar multiplier (>=0)
 *  - w_diag_final_  : final per-row weights (= weight_scalar_ * w_diag_user_)
 *  - W_             : diagonal matrix built from w_diag_final_
 */
class TaskBase
{
public:
  enum class GainType { Acceleration = 0, Force = 1 };

  explicit TaskBase(std::string id);
  virtual ~TaskBase() = default;

  // identity / state
  const std::string& id() const { return id_; }
  
  bool enabled() const { return enabled_; }
  void setEnabled(bool en) { enabled_ = en; }

  // ----- lambda parameters (task-specific semantics) -----
  void setLambda(double l1, double l2 = 0.0);
  double getLambda()  const { return lambda1_; } // Backward-compatible alias.
  double getLambda1() const { return lambda1_; }
  double getLambda2() const { return lambda2_; }

  // ----- weights -----
  // Scalar multiplies user row-weights: w_final = weight_scalar * w_user
  void setWeightScalar(double w);
  double getWeightScalar() const { return weight_scalar_; }

  // Per-row weights (diagonal). These are the preferred APIs.
  void setWeightDiag(const Eigen::VectorXd& w_diag);
  void setWeightDiag(double w); // constant diag

  // Backward-compatible aliases.
  void setWeight(const Eigen::VectorXd& w_diag) { setWeightDiag(w_diag); }
  void setWeight(const Eigen::MatrixXd& Wdiag);

  // Returns the diagonal matrix built from final row weights.
  Eigen::MatrixXd getWeight() const { return W_; }
  Eigen::MatrixXd WdiagMatrix() const { return W_; }

  // Preferred getter used by IDProblem
  const Eigen::MatrixXd& W() const { return W_; }

  // Final row weights (after scalar multiplier)
  const Eigen::VectorXd& wDiag() const { return w_diag_final_; }

  // ----- gains (optional) -----
  void setKp(const Eigen::MatrixXd& Kp);
  void setKd(const Eigen::MatrixXd& Kd);
  const Eigen::MatrixXd& getKp() const { return Kp_; }
  const Eigen::MatrixXd& getKd() const { return Kd_; }

  // ----- LSQ term -----
  const Eigen::MatrixXd& A() const { return A_; }
  const Eigen::VectorXd& b() const { return b_; }

  int rows() const { return static_cast<int>(b_.size()); }
  int cols() const { return static_cast<int>(A_.cols()); }

  // Update term (task-specific)
  virtual void update() = 0;
  // Compatibility overload for existing call sites.
  void update(const Eigen::VectorXd& /*x*/) { update(); }

  // Optional: reset internal caches (references, integrators, etc.)
  virtual bool reset() { return true; }

  // Gain type (used by some tasks)
  void setGainType(GainType t) { gain_type_ = t; }
  GainType getGainType() const { return gain_type_; }

protected:
  // Resize LSQ buffers and weight buffers coherently
  void resize(int rows, int cols);

  static inline bool isFinite(double v) { return std::isfinite(v); }

  // Build w_diag_final_ and W_ from current user weights and scalar
  void finalizeWeights();

  // Set default row weights to ones(rows)
  void setDefaultRowWeights(int rows);

protected:
  std::string id_;
  bool enabled_{true};

  GainType gain_type_{GainType::Acceleration};
  double lambda1_{0.0};
  double lambda2_{0.0};

  // user-provided row weights (>=0), size = rows
  Eigen::VectorXd w_diag_user_;
  // final row weights used by QP (after scalar multiplier)
  Eigen::VectorXd w_diag_final_;
  // scalar multiplier
  double weight_scalar_{1.0};

  // diagonal weight matrix built from w_diag_final_
  Eigen::MatrixXd W_;

  Eigen::MatrixXd Kp_;
  Eigen::MatrixXd Kd_;

  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
};

} // namespace wolf_wbid

#endif // WOLF_WBID_TASK_BASE_H
