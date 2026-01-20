/**
WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
*/

#ifndef WOLF_WBID_COM_TASK_H
#define WOLF_WBID_COM_TASK_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace wolf_wbid {

class QuadrupedRobot;
class IDVariables;

class ComTask
{
public:
  using Ptr = std::shared_ptr<ComTask>;

  ComTask(const std::string& task_id,
          QuadrupedRobot& robot,
          const IDVariables& vars);

  virtual ~ComTask() = default;

  // --- Update ---
  void update(const Eigen::VectorXd& x);

  // --- Task form: minimize || A x - b ||_W ---
  const Eigen::MatrixXd& A() const { return A_; }
  const Eigen::VectorXd& b() const { return b_; }
  const Eigen::Matrix3d& W() const { return W_; }

  // --- Metadata ---
  const std::string& getTaskID() const { return task_id_; }
  const std::string& getBaseLink() const { return base_link_; }      // kept for ROS msgs compatibility
  const std::string& getDistalLink() const { return distal_link_; }  // "CoM"

  // --- References ---
  void setReference(const Eigen::Vector3d& pos_ref);
  void setReference(const Eigen::Vector3d& pos_ref,
                    const Eigen::Vector3d& vel_ref);
  void setReference(const Eigen::Vector3d& pos_ref,
                    const Eigen::Vector3d& vel_ref,
                    const Eigen::Vector3d& acc_ref);

  void getReference(Eigen::Vector3d& pos_ref) const { pos_ref = pos_ref_; }
  void getReference(Eigen::Vector3d& pos_ref, Eigen::Vector3d& vel_ref) const
  {
    pos_ref = pos_ref_;
    vel_ref = vel_ref_;
  }

  const Eigen::Vector3d& getCachedVelocityReference() const { return vel_ref_cached_; }
  const Eigen::Vector3d& getCachedAccelerationReference() const { return acc_ref_cached_; }

  // --- Actuals (valid after update) ---
  void getActualPose(Eigen::Vector3d& pos) const { pos = pos_current_; }
  void getActualVelocity(Eigen::Vector3d& vel) const { vel = vel_current_; }

  // --- Errors (valid after update) ---
  const Eigen::Vector3d& getPosError() const { return pos_error_; }
  const Eigen::Vector3d& getVelError() const { return vel_error_; }

  // --- Gains ---
  void setLambda(double lambda);
  void setLambda(double lambda1, double lambda2);
  double getLambda() const { return lambda1_; }
  double getLambda2() const { return lambda2_; }

  void setKp(const Eigen::Matrix3d& Kp) { Kp_ = Kp; }
  void setKd(const Eigen::Matrix3d& Kd) { Kd_ = Kd; }
  const Eigen::Matrix3d& getKp() const { return Kp_; }
  const Eigen::Matrix3d& getKd() const { return Kd_; }

  void setGains(const Eigen::Matrix3d& Kp, const Eigen::Matrix3d& Kd)
  {
    Kp_ = Kp;
    Kd_ = Kd;
  }

  // --- Weight ---
  void setWeight(const Eigen::Matrix3d& W) { W_ = W; }
  const Eigen::Matrix3d& getWeight() const { return W_; }

  // --- Reset ---
  virtual bool reset();

private:
  void resetReference();

  std::string task_id_;
  QuadrupedRobot& robot_;
  const IDVariables& vars_;

  // For compatibility with your ROS msgs / old wrappers
  std::string distal_link_{"CoM"};
  std::string base_link_{"world"};

  // Jacobian terms
  Eigen::MatrixXd Jcom_;              // 3 x nvars_q (same dofs as robot_)
  Eigen::Vector3d dJcomQdot_;         // 3

  // reference/current
  Eigen::Vector3d pos_ref_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_current_{Eigen::Vector3d::Zero()};

  Eigen::Vector3d vel_ref_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_ref_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_ref_cached_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc_ref_cached_{Eigen::Vector3d::Zero()};

  // errors
  Eigen::Vector3d pos_error_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_current_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_error_{Eigen::Vector3d::Zero()};

  // gains
  double lambda1_{100.0};
  double lambda2_{2.0 * std::sqrt(100.0)};
  Eigen::Matrix3d Kp_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d Kd_{Eigen::Matrix3d::Identity()};

  // weight
  Eigen::Matrix3d W_{Eigen::Matrix3d::Identity()};

  // task matrices (A x ~= b)
  Eigen::MatrixXd A_; // 3 x dim(x)
  Eigen::VectorXd b_; // 3
};

} // namespace wolf_wbid

#endif // WOLF_WBID_COM_TASK_H
