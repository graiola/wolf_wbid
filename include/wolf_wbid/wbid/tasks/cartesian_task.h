/**
WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
*/

#ifndef WOLF_WBID_CARTESIAN_TASK_H
#define WOLF_WBID_CARTESIAN_TASK_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace wolf_wbid {

class QuadrupedRobot;
class IDVariables;

class CartesianTask
{
public:
  using Ptr = std::shared_ptr<CartesianTask>;

  enum class GainType { Acceleration = 0, Force = 1 };

  CartesianTask(const std::string& task_id,
                QuadrupedRobot& robot,
                const std::string& distal_link,
                const std::string& base_link,
                const IDVariables& vars);

  virtual ~CartesianTask() = default;

  // --- Update ---
  void update(const Eigen::VectorXd& x);

  // --- Task form: minimize || A x - b ||_W ---
  const Eigen::MatrixXd& A() const { return A_; }
  const Eigen::VectorXd& b() const { return b_; }
  const Eigen::Matrix<double,6,6>& W() const { return W_; }

  // --- Links ---
  const std::string& getBaseLink() const { return base_link_; }
  const std::string& getDistalLink() const { return distal_link_; }

  bool setBaseLink(const std::string& base_link);
  bool setDistalLink(const std::string& distal_link);

  // --- References ---
  void setReference(const Eigen::Affine3d& pose_ref);
  void setReference(const Eigen::Affine3d& pose_ref, const Eigen::Matrix<double,6,1>& vel_ref);
  void setReference(const Eigen::Affine3d& pose_ref,
                    const Eigen::Matrix<double,6,1>& vel_ref,
                    const Eigen::Matrix<double,6,1>& acc_ref);

  void setVirtualForce(const Eigen::Matrix<double,6,1>& virtual_force_ref);

  void getReference(Eigen::Affine3d& pose_ref) const;
  void getReference(Eigen::Affine3d& pose_ref, Eigen::Matrix<double,6,1>& vel_ref) const;

  const Eigen::Matrix<double,6,1>& getCachedVelocityReference() const { return vel_ref_cached_; }
  const Eigen::Matrix<double,6,1>& getCachedAccelerationReference() const { return acc_ref_cached_; }
  const Eigen::Matrix<double,6,1>& getCachedVirtualForceReference() const { return virtual_force_ref_cached_; }

  // --- Actuals (valid after update) ---
  void getActualPose(Eigen::Affine3d& pose) const { pose = pose_current_; }
  void getActualTwist(Eigen::Matrix<double,6,1>& twist) const { twist = vel_current_; }

  // --- Errors (valid after update) ---
  const Eigen::Matrix<double,6,1>& getError() const { return pose_error_; }
  const Eigen::Matrix<double,6,1>& getVelocityError() const { return vel_error_; }

  // --- Gains ---
  void setGainType(GainType t) { gain_type_ = t; }
  GainType getGainType() const { return gain_type_; }

  void setLambda(double lambda);
  void setLambda(double lambda1, double lambda2);
  double getLambda() const { return lambda1_; }
  double getLambda2() const { return lambda2_; }

  void setOrientationGain(double g);
  double getOrientationErrorGain() const { return orientation_gain_; }

  void setKp(const Eigen::Matrix<double,6,6>& Kp) { Kp_ = Kp; }
  void setKd(const Eigen::Matrix<double,6,6>& Kd) { Kd_ = Kd; }
  const Eigen::Matrix<double,6,6>& getKp() const { return Kp_; }
  const Eigen::Matrix<double,6,6>& getKd() const { return Kd_; }

  // --- Weight ---
  void setWeight(const Eigen::Matrix<double,6,6>& W) { W_ = W; }
  const Eigen::Matrix<double,6,6>& getWeight() const { return W_; }

  // --- Reset ---
  virtual bool reset();

private:
  void resetReference();
  void computeOrientationError(const Eigen::Matrix3d& R_des, const Eigen::Matrix3d& R_act, Eigen::Vector3d& e) const;
  void computeCartesianInertiaInverse(); // for GainType::Force

  bool readRobotState();

  // cached robot state (size = robot_.getJointNum())
  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;

  std::string task_id_;
  QuadrupedRobot& robot_;
  const IDVariables& vars_;

  std::string base_link_;
  std::string distal_link_;

  GainType gain_type_{GainType::Acceleration};

  // dynamics / kinematics buffers
  Eigen::MatrixXd J_;                 // 6 x n
  Eigen::Matrix<double,6,1> jdotqdot_;

  // reference/current
  Eigen::Affine3d pose_ref_{Eigen::Affine3d::Identity()};
  Eigen::Affine3d pose_current_{Eigen::Affine3d::Identity()};

  Eigen::Matrix<double,6,1> vel_ref_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> acc_ref_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> vel_ref_cached_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> acc_ref_cached_{Eigen::Matrix<double,6,1>::Zero()};

  Eigen::Matrix<double,6,1> virtual_force_ref_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> virtual_force_ref_cached_{Eigen::Matrix<double,6,1>::Zero()};

  // errors
  Eigen::Matrix<double,6,1> pose_error_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> vel_current_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> vel_error_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Vector3d orientation_error_{Eigen::Vector3d::Zero()};

  // gains
  double lambda1_{100.0};
  double lambda2_{2.0 * std::sqrt(100.0)};
  double orientation_gain_{1.0};
  Eigen::Matrix<double,6,6> Kp_{Eigen::Matrix<double,6,6>::Identity()};
  Eigen::Matrix<double,6,6> Kd_{Eigen::Matrix<double,6,6>::Identity()};

  // weight
  Eigen::Matrix<double,6,6> W_{Eigen::Matrix<double,6,6>::Identity()};

  // Force-mode buffers
  Eigen::Matrix<double,6,6> Mi_{Eigen::Matrix<double,6,6>::Zero()};
  Eigen::MatrixXd Bi_;          // n x n
  Eigen::MatrixXd tmp6xn_;      // 6 x n

  // task matrices (A x ~= b)
  Eigen::MatrixXd A_; // 6 x dim(x)
  Eigen::VectorXd b_; // 6
};

} // namespace wolf_wbid

#endif
