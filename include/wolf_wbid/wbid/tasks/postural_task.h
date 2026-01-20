/**
 * WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
 */

#ifndef WOLF_WBID_POSTURAL_TASK_H
#define WOLF_WBID_POSTURAL_TASK_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <atomic>
#include <stdexcept>
#include <cmath>

#include <wolf_wbid/wbid/id_variables.h>
#include <wolf_wbid/quadruped_robot.h>

namespace wolf_wbid {

class PosturalTask
{
public:
  using Ptr = std::shared_ptr<PosturalTask>;

  enum class GainType { Acceleration = 0, Force = 1 };

  PosturalTask(const std::string& task_id,
               QuadrupedRobot& model,
               const IDVariables& idvars,
               double period = 0.001);

  virtual ~PosturalTask() = default;

  /** Main update entry point (solver calls this) */
  void update(const Eigen::VectorXd& x);

  /** Reset reference to current posture */
  virtual bool reset();

  /** Task interface: minimize || W^(1/2) (A x - b) ||^2 */
  const Eigen::MatrixXd& A() const { return A_; }
  const Eigen::VectorXd& b() const { return b_; }
  const Eigen::MatrixXd& W() const { return W_; }

  /** Scalar weight (diag) helper */
  void setWeightDiag(double w);
  double getWeightDiag() const { return weight_diag_; }

  /** Lambdas */
  void setLambda(double lambda1);                 // sets lambda2 = 2*sqrt(lambda1)
  void setLambda(double lambda1, double lambda2);
  double getLambda1() const { return lambda1_; }
  double getLambda2() const { return lambda2_; }

  /** Gains */
  void setGainType(GainType t) { gain_type_ = t; }
  GainType getGainType() const { return gain_type_; }

  void setKp(const Eigen::MatrixXd& Kp);
  void setKd(const Eigen::MatrixXd& Kd);
  void setGains(const Eigen::MatrixXd& Kp, const Eigen::MatrixXd& Kd);
  const Eigen::MatrixXd& getKp() const { return Kp_; }
  const Eigen::MatrixXd& getKd() const { return Kd_; }

  /** References */
  void setReference(const Eigen::VectorXd& qref);
  void setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& qdot_ref);
  void setReference(const Eigen::VectorXd& qref, const Eigen::VectorXd& qdot_ref, const Eigen::VectorXd& qddot_ref);

  const Eigen::VectorXd& refQ() const { return qref_; }
  const Eigen::VectorXd& refQdotCached() const { return qdot_ref_cached_; }
  const Eigen::VectorXd& refQddotCached() const { return qddot_ref_cached_; }

  /** Actual state + errors (cached after update) */
  const Eigen::VectorXd& actualQ() const { return q_; }
  const Eigen::VectorXd& actualQdot() const { return qdot_; }
  const Eigen::VectorXd& posError() const { return position_error_; }
  const Eigen::VectorXd& velError() const { return velocity_error_; }

  /** Desired acceleration computed at last update */
  const Eigen::VectorXd& desiredQddot() const { return qddot_d_; }

  /** Joint names for publishing */
  const std::vector<std::string>& jointNames() const { return joint_names_; }

  /** Cost */
  double computeCost(const Eigen::VectorXd& x) const;
  double costLast() const { return cost_last_; }

  const std::string& id() const { return task_id_; }
  int taskSize() const { return task_size_; }      // nq
  int xSize() const { return x_size_; }            // full ID x size

protected:
  /** Extension hook: wrapper may override to apply buffered params then call base */
  virtual void _update(const Eigen::VectorXd& x);

  /** Build selector A = [I 0] once */
  void buildSelectionMatrix();

protected:
  std::string task_id_;
  QuadrupedRobot& model_;
  const IDVariables& idvars_;
  double period_;

  int task_size_{0}; // nq
  int x_size_{0};    // idvars.size()

  // A, b, W for least squares
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  Eigen::MatrixXd W_;

  // gains & settings
  GainType gain_type_{GainType::Acceleration};
  double lambda1_{10.0};
  double lambda2_{2.0 * std::sqrt(10.0)};
  double weight_diag_{1.0};

  Eigen::MatrixXd Kp_;
  Eigen::MatrixXd Kd_;

  // references (live) and cached (published)
  Eigen::VectorXd qref_;
  Eigen::VectorXd qdot_ref_;
  Eigen::VectorXd qddot_ref_;

  Eigen::VectorXd qdot_ref_cached_;
  Eigen::VectorXd qddot_ref_cached_;

  // actual state and errors
  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  Eigen::VectorXd position_error_;
  Eigen::VectorXd velocity_error_;

  // desired accel
  Eigen::VectorXd qddot_d_;

  // inertia inverse (for GainType::Force)
  Eigen::MatrixXd Mi_;

  // names
  std::vector<std::string> joint_names_;

  // cached cost
  double cost_last_{0.0};
};

} // namespace wolf_wbid

#endif // WOLF_WBID_POSTURAL_TASK_H
