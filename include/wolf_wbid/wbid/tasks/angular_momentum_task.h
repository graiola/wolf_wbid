// ============================================================================
// File: include/wolf_wbid/wbid/tasks/angular_momentum_task.h
// OpenSoT-free Angular Momentum task (WoLF)
// ============================================================================

/**
WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
*/

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

namespace wolf_wbid {

class QuadrupedRobot;
class IDVariables;

/**
 * Task form: minimize || A x - b ||_W
 *
 * Uses centroidal momentum dynamics:
 *   hdot = CMM * qddot + CMMdotQdot
 * with h = [p; L], we use only angular part:
 *   Ldot = CMM_bottom * qddot + (CMMdotQdot)_ang
 *
 * Tracking law (OpenSoT-like):
 *   Ldot_ref = Ldot_d + lambda * K * (L_d - L)
 */
class AngularMomentumTask
{
public:
  using Ptr = std::shared_ptr<AngularMomentumTask>;

  AngularMomentumTask(const std::string& task_id,
                      QuadrupedRobot& robot,
                      const IDVariables& vars);

  void update(const Eigen::VectorXd& x);

  // LS form
  const Eigen::MatrixXd& A() const { return A_; }
  const Eigen::VectorXd& b() const { return b_; }
  const Eigen::Matrix3d& W() const { return W_; }

  // Meta
  const std::string& getBaseLink() const { return base_link_; }
  const std::string& getDistalLink() const { return distal_link_; }

  // Gains
  void setLambda(double lambda);
  double getLambda() const { return lambda_; }

  void setMomentumGain(const Eigen::Matrix3d& K) { K_ = K; }
  const Eigen::Matrix3d& getMomentumGain() const { return K_; }

  void setWeight(const Eigen::Matrix3d& W) { W_ = W; }
  const Eigen::Matrix3d& getWeight() const { return W_; }

  // References
  void setReference(const Eigen::Vector3d& desiredAngularMomentum);
  void setReference(const Eigen::Vector3d& desiredAngularMomentum,
                    const Eigen::Vector3d& desiredAngularMomentumVariation);

  void getReference(Eigen::Vector3d& desiredAngularMomentum) const;
  void getReference(Eigen::Vector3d& desiredAngularMomentum,
                    Eigen::Vector3d& desiredAngularMomentumVariation) const;

  const Eigen::Vector3d& getCachedDesiredAngularMomentum() const { return L_d_cached_; }
  const Eigen::Vector3d& getCachedDesiredAngularMomentumRate() const { return Ldot_d_cached_; }

  // Actuals (valid after update)
  const Eigen::Vector3d& getActualAngularMomentum() const { return L_ang_; }
  const Eigen::Vector3d& getAngularMomentumBias() const { return Ldot_bias_; }

  virtual bool reset();

private:
  std::string task_id_;
  QuadrupedRobot& robot_;
  const IDVariables& vars_;

  std::string base_link_{"CoM"};
  std::string distal_link_{"CoM"};

  bool is_init_{false};

  double lambda_{1.0};
  Eigen::Matrix3d K_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d W_{Eigen::Matrix3d::Identity()};

  // CMM (6 x n) and bias (6)
  Eigen::MatrixXd Mom_;
  Eigen::Matrix<double,6,1> CMMdotQdot_;

  // centroidal momentum (6) from your API: [linear; angular]
  Eigen::Matrix<double,6,1> h_;
  Eigen::Vector3d L_ang_;
  Eigen::Vector3d Ldot_bias_;

  // references (reset to zero after update)
  Eigen::Vector3d L_d_;
  Eigen::Vector3d Ldot_d_;

  // cached references for publishing
  Eigen::Vector3d L_d_cached_;
  Eigen::Vector3d Ldot_d_cached_;

  Eigen::Vector3d Ldot_ref_;

  // task matrices
  Eigen::MatrixXd A_; // 3 x dim(x)
  Eigen::VectorXd b_; // 3
};

} // namespace wolf_wbid
