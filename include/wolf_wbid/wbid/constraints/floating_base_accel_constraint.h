#pragma once

#include <wolf_wbid/wbid/constraints/constraint_base.h>
#include <wolf_wbid/wbid/id_variables.h>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <stdexcept>

namespace wolf_wbid {

/**
 * Constraint on selected floating-base accelerations inside qddot block.
 *
 * Base ordering assumed (as in your debug print):
 *   qddot_base = [x y z roll pitch yaw]
 *
 * You provide a list of indices in [0..5] to constrain.
 */
class FloatingBaseAccelConstraint : public IConstraint
{
public:
  enum class Mode {
    EQUALITY,      // qi == 0
    LOWER_BOUND,   // qi >= -eps
    UPPER_BOUND,   // qi <= +eps
    BOX            // -eps <= qi <= +eps
  };

  FloatingBaseAccelConstraint(const std::string& name,
                              const IDVariables& vars,
                              const std::vector<int>& base_indices,
                              Mode mode,
                              double eps)
  : name_(name),
    vars_(vars),
    base_idx_(base_indices),
    mode_(mode),
    eps_(eps)
  {
    if(base_idx_.empty())
      throw std::runtime_error("FloatingBaseAccelConstraint: empty base_indices");

    for(int i : base_idx_) {
      if(i < 0 || i > 5)
        throw std::runtime_error("FloatingBaseAccelConstraint: base index out of [0..5]");
    }

    enabled_ = true;
    // sizes will be finalized in update() once we know qp.n (vars size)
  }

  // --- IConstraint API -----------------------------------------------------
  const std::string& name() const override { return name_; }
  bool enabled() const override { return enabled_; }
  void setEnabled(bool en) { enabled_ = en; }

  int rows() const override { return static_cast<int>(base_idx_.size()); }
  int cols() const override { return nvars_; }

  const Eigen::MatrixXd& A()  const override { return A_;  }
  const Eigen::VectorXd& lA() const override { return lA_; }
  const Eigen::VectorXd& uA() const override { return uA_; }

  // No variable bounds in this constraint
  // (so it will be treated as "linear constraint" only)
  void update(const Eigen::VectorXd& /*x*/) override
  {
    if(!enabled_) {
      // keep consistent sizes anyway, for safer stacking
      ensureSized();
      A_.setZero();
      lA_.setConstant(-kBig());
      uA_.setConstant( kBig());
      return;
    }

    ensureSized();
    A_.setZero();

    const auto& qb = vars_.qddotBlock();
    if(qb.dim < 6)
      throw std::runtime_error("FloatingBaseAccelConstraint: qddotBlock dim < 6 (not floating base?)");

    // Build A selecting desired base components
    for(int r = 0; r < rows(); ++r)
    {
      const int bi = base_idx_[r];               // 0..5
      const int col = qb.offset + bi;            // absolute in x
      if(col < 0 || col >= nvars_)
        throw std::runtime_error("FloatingBaseAccelConstraint: qb.offset out of bounds");

      A_(r, col) = 1.0;
    }

    // bounds depending on mode
    switch(mode_)
    {
      case Mode::EQUALITY:
        lA_.setZero();
        uA_.setZero();
        break;

      case Mode::LOWER_BOUND:
        lA_.setConstant(-eps_); // qi >= -eps
        uA_.setConstant( kBig());
        break;

      case Mode::UPPER_BOUND:
        lA_.setConstant(-kBig());
        uA_.setConstant( eps_); // qi <= +eps
        break;

      case Mode::BOX:
        lA_.setConstant(-eps_);
        uA_.setConstant( eps_);
        break;
    }
  }

  // --- configuration helpers ----------------------------------------------
  void setMode(Mode m) { mode_ = m; }
  void setEps(double e) { eps_ = e; }

  void setBaseIndices(const std::vector<int>& idx)
  {
    if(idx.empty()) throw std::runtime_error("FloatingBaseAccelConstraint: empty indices");
    base_idx_ = idx;
    // force resize on next update
    nvars_ = -1;
  }

private:
  static inline double kBig() { return 1.0e20; }

  void ensureSized()
  {
    const int nv = vars_.size();
    if(nv <= 0) throw std::runtime_error("FloatingBaseAccelConstraint: vars size <= 0");
    if(nvars_ == nv && A_.rows() == rows() && A_.cols() == nv) return;

    nvars_ = nv;
    A_.resize(rows(), nvars_);
    lA_.resize(rows());
    uA_.resize(rows());
  }

private:
  std::string name_;
  const IDVariables& vars_;

  std::vector<int> base_idx_;   // subset of [0..5]
  Mode mode_;
  double eps_{0.0};

  bool enabled_{true};

  int nvars_{-1};
  Eigen::MatrixXd A_;
  Eigen::VectorXd lA_;
  Eigen::VectorXd uA_;
};

} // namespace wolf_wbid
