/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

// wolf_wbid/wbid/constraint_base.h
#pragma once

#include <Eigen/Dense>
#include <string>

namespace wolf_wbid {

/** @brief Large finite value used as practical infinity for bounds. */
inline constexpr double kInf = 1.0e20;

/**
 * @brief Interface for linear constraints of the form `lA <= A*x <= uA`.
 */
class IConstraint
{
public:
  virtual ~IConstraint() = default;

  virtual const std::string& name() const = 0;

  virtual bool enabled() const = 0;
  virtual void setEnabled(bool en) = 0;

  // Updates internal matrices and bounds.
  virtual void update(const Eigen::VectorXd& x) = 0;

  // Linear constraint dimensions and data.
  virtual int rows() const = 0;
  virtual int cols() const = 0;

  virtual const Eigen::MatrixXd& A() const = 0;
  virtual const Eigen::VectorXd& lA() const = 0;
  virtual const Eigen::VectorXd& uA() const = 0;
};

/**
 * @brief Convenience base class for constraints with optional variable bounds.
 */
class ConstraintBase : public IConstraint
{
public:
  explicit ConstraintBase(std::string name)
  : name_(std::move(name))
  {}

  const std::string& name() const override { return name_; }

  bool enabled() const override { return enabled_; }
  void setEnabled(bool en) override { enabled_ = en; }

  int rows() const override { return static_cast<int>(A_.rows()); }
  int cols() const override { return static_cast<int>(A_.cols()); }

  const Eigen::MatrixXd& A() const override { return A_; }
  const Eigen::VectorXd& lA() const override { return lA_; }
  const Eigen::VectorXd& uA() const override { return uA_; }

  // Optional variable bounds (size nvars). Empty means no bound contribution.
  const Eigen::VectorXd& l() const { return l_; }
  const Eigen::VectorXd& u() const { return u_; }
  bool hasBounds() const { return (l_.size() > 0) && (u_.size() > 0); }

protected:
  void resizeLinear(int m, int n)
  {
    A_.setZero(m, n);
    lA_.setZero(m);
    uA_.setZero(m);
  }

  void setEq(const Eigen::VectorXd& b)
  {
    lA_ = b;
    uA_ = b;
  }

  void resizeBounds(int nvars)
  {
    l_.setConstant(nvars, -kInf);
    u_.setConstant(nvars,  kInf);
  }

  Eigen::VectorXd l_;
  Eigen::VectorXd u_;

  std::string name_;
  bool enabled_{true};

  Eigen::MatrixXd A_;
  Eigen::VectorXd lA_;
  Eigen::VectorXd uA_;
};

} // namespace wolf_wbid
