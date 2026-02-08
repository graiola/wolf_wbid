/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
**/

#ifndef WOLF_WBID_QUADRUPED_ROBOT_RBDL_H
#define WOLF_WBID_QUADRUPED_ROBOT_RBDL_H

// RBDL
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

// SRDF
#include <srdfdom/model.h>

// WoLF
#include <wolf_wbid/core/quadruped_robot.h>
#include <wolf_controller_utils/srdf_parser.h>

namespace wolf_wbid
{

class QuadrupedRobotRBDL : public QuadrupedRobot
{
public:
  explicit QuadrupedRobotRBDL(const std::string &robot_name,
                              const std::string& urdf,
                              const std::string& srdf);

  bool update( bool update_position = true,
               bool update_velocity = true,
               bool update_desired_acceleration = true ) override;

  const std::vector<std::string>& getFootNames() const override;
  const std::vector<std::string>& getLegNames() const override;
  const std::vector<std::string>& getHipNames() const override;
  const std::vector<std::string>& getJointNames() const override;
  const std::vector<std::string>& getArmNames() const override;
  const std::vector<std::string>& getEndEffectorNames() const override;
  const std::vector<std::string>& getContactNames() const override;
  const std::vector<std::string>& getLimbNames() const override;
  const std::string& getBaseLinkName() const override;
  const std::string& getImuSensorName() const override;
  const std::string& getRobotName() const override;
  const std::string& getRobotModelName() const override;
  const std::vector<std::string>& getEnabledJointNames() const override;
  Eigen::VectorXd getLegJointValues(const Eigen::VectorXd& joints) override;
  Eigen::VectorXd getArmJointValues(const Eigen::VectorXd& joints) override;

  const std::vector<unsigned int> &getLimbJointsIds(const std::string& limb_name) override;

  const unsigned int& getNumberArms() const override;
  const unsigned int& getNumberLegs() const override;

  const double& getBaseLength() const override;
  const double& getBaseWidth() const override;

  bool isRobotFalling(const double& dt) const override;

  void getFloatingBasePositionInertia(Eigen::Matrix3d& M) override;
  void getFloatingBaseOrientationInertia(Eigen::Matrix3d& M) override;
  void getLimbInertia(const std::string& limb_name, Eigen::MatrixXd& M) override;
  void getLimbInertiaInverse(const std::string& limb_name, Eigen::MatrixXd& Mi) override;

  const Eigen::Matrix3d& getBaseRotationInHf() const override;
  const Eigen::Matrix3d& getHfRotationInWorld() const override;
  const Eigen::Matrix3d& getBaseRotationInWorld() const override;
  const Eigen::Vector3d& getBaseRotationInWorldRPY() const override;
  const double& getBaseYawInWorld() const override;

  void getCOM(Eigen::Vector3d& p_W) const override;
  void getCOMVelocity(Eigen::Vector3d& v_W) const override;
  void getCOMJacobian(Eigen::MatrixXd& Jcom) const override;
  void getCentroidalMomentum(Eigen::Vector6d& h) const override;
  void getCentroidalMomentumMatrix(Eigen::MatrixXd& CMM) const override;
  void getCentroidalMomentumMatrix(Eigen::MatrixXd& CMM, Eigen::Vector6d& CMMdotQdot) const override;
  bool getVelocityTwist(const std::string& link, Eigen::Vector6d& twist) const override;
  bool getVelocityTwist(const std::string& distal, const std::string& base, Eigen::Vector6d& twist) const override;
  bool computeRelativeJdotQdot(const std::string& target_link, const std::string& base_link, Eigen::Vector6d& jdotqdot) const override;
  bool computeJdotQdot(const std::string& link, const Eigen::Vector3d& point, Eigen::Vector6d& jdotqdot) const override;
  bool getPose(const std::string& source_frame, Eigen::Affine3d& pose) const override;
  bool getPose(const std::string& source_frame, const std::string& target_frame, Eigen::Affine3d& pose) const override;
  bool getJacobian(const std::string& link_name, Eigen::MatrixXd& J) const override;
  bool getJacobian(const std::string& link_name, const std::string& target_frame, Eigen::MatrixXd& J) const override;

  const std::map<std::string, Eigen::Vector3d>& getFeetPositionInWorld() const override;
  const std::map<std::string, Eigen::Vector3d>& getFeetPositionInBase() const override;
  const std::map<std::string, Eigen::Affine3d>& getFeetPoseInWorld() const override;
  const std::map<std::string, Eigen::Affine3d>& getFeetPoseInBase() const override;

  Eigen::Vector3d& getFootPositionInWorld(const std::string &name) override;
  Eigen::Vector3d& getFootPositionInBase(const std::string &name) override;
  Eigen::Affine3d& getFootPoseInWorld(const std::string &name) override;
  Eigen::Affine3d& getFootPoseInBase(const std::string &name) override;

  const std::map<std::string, Eigen::Vector3d>& getEndEffectorsPositionInWorld() const override;
  const std::map<std::string, Eigen::Vector3d>& getEndEffectorsPositionInBase() const override;
  const std::map<std::string, Eigen::Affine3d>& getEndEffectorsPoseInWorld() const override;
  const std::map<std::string, Eigen::Affine3d>& getEndEffectorsPoseInBase() const override;

  Eigen::Vector3d& getEndEffectorPositionInWorld(const std::string &name) override;
  Eigen::Vector3d& getEndEffectorPositionInBase(const std::string &name) override;
  Eigen::Affine3d& getEndEffectorPoseInWorld(const std::string &name) override;
  Eigen::Affine3d& getEndEffectorPoseInBase(const std::string &name) override;

  const Eigen::Affine3d& getBasePoseInWorld() const override;

  std::vector<bool> checkJointVelocities(Eigen::VectorXd &qdot) override;
  bool clampJointVelocities(Eigen::VectorXd &qdot) override;
  bool clampJointPositions(Eigen::VectorXd &q) override;
  bool clampJointEfforts(Eigen::VectorXd &tau) override;

  const Eigen::VectorXd& getStandUpJointPostion() override;
  const Eigen::VectorXd& getStandDownJointPostion() override;
  const double& getStandUpHeight() override;
  const double& getStandDownHeight() override;

  bool getPose(const Eigen::VectorXd& q, const std::string& source_frame, const std::string& target_frame, Eigen::Affine3d& pose) override;
  bool getPose(const Eigen::VectorXd& q, const std::string& source_frame, Eigen::Affine3d& pose) override;
  bool getRelativeJacobian(const Eigen::VectorXd& q, const std::string& target_link_name, const std::string& base_link_name, Eigen::MatrixXd& J) override;
  bool getTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const std::string& source_frame, Eigen::Vector6d& twist) override;
  bool getTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const std::string& distal, const std::string& base, Eigen::Vector6d& twist_rel) override;
  bool getJacobian(const Eigen::VectorXd& q, const std::string& link_name, Eigen::MatrixXd& J) override;
  bool getJacobian(const Eigen::VectorXd& q, const std::string& link_name, const std::string& target_frame, Eigen::MatrixXd& J) override;

  double getCurrentHeight() override;

  int getJointNum() const override;
  double getMass() const override;
  int getJointIndex(const std::string& joint_name) const override;
  bool getJointPosition(Eigen::VectorXd& q) const override;
  bool getJointVelocity(Eigen::VectorXd& qd) const override;
  bool getJointEffort(Eigen::VectorXd& tau) const override;
  const Eigen::VectorXd& getJointPositions() const override;
  const Eigen::VectorXd& getJointVelocities() const override;
  const Eigen::VectorXd& getJointEfforts() const override;
  bool setJointPosition(const Eigen::VectorXd& q) override;
  bool setJointVelocity(const Eigen::VectorXd& qd) override;
  bool setJointEffort(const Eigen::VectorXd& tau) override;
  bool setJointAcceleration(const Eigen::VectorXd& qddot) override;
  bool setJointPosition(int i, double q) override;
  bool setJointVelocity(int i, double qd) override;
  bool setJointEffort(int i, double tau) override;

  bool getFloatingBasePose(Eigen::Affine3d& pose) const override;
  bool setFloatingBasePose(const Eigen::Affine3d& pose) override;
  bool setFloatingBaseOrientation(const Eigen::Matrix3d& world_R_base) override;
  bool setFloatingBaseAngularVelocity(const Eigen::Vector3d& w) override;
  bool setFloatingBaseState(const Eigen::Affine3d& pose, const Eigen::Vector6d& twist) override;

  void computeInverseDynamics(Eigen::VectorXd& tau) const override;
  bool isFloatingBase() const override;

  void getInertiaMatrix(Eigen::MatrixXd& M) const override;
  void getInertiaInverse(Eigen::MatrixXd& Minv) const override;
  void computeNonlinearTerm(Eigen::VectorXd& h) const override;
  void computeGravityCompensation(Eigen::VectorXd& g) const override;

  bool getOrientation(const std::string& frame, Eigen::Matrix3d& R) const override;
  bool getOrientation(const std::string& source_frame, const std::string& target_frame, Eigen::Matrix3d& R) const override;

  const urdf::ModelInterface& getUrdf() const override;
  const std::string& getUrdfString() const override;
  const std::string& getSrdfString() const override;
  bool getJointLimits(Eigen::VectorXd& qmin, Eigen::VectorXd& qmax) const override;
  bool getVelocityLimits(Eigen::VectorXd& qdot_max) const override;
  bool getEffortLimits(Eigen::VectorXd& tau_max) const override;

private:
  int linkId(const std::string& link_name) const;
  bool resolveFixedLink(const std::string& link_name, std::string& ancestor_link, Eigen::Affine3d& T_ancestor_to_link) const;
  bool getJacobian(const std::string& link_name, const Eigen::Vector3d& point, Eigen::MatrixXd& J) const;
  bool loadSrdfState(const std::string& state_name, Eigen::VectorXd& q) const;

  wolf_controller_utils::SRDFParser parser_;
  srdf::Model srdf_model_;
  urdf::Model urdf_model_;
  std::string urdf_string_;
  std::string srdf_string_;

  std::vector<std::string> foot_names_;
  std::vector<std::string> hip_names_;
  std::vector<std::string> leg_names_;
  std::vector<std::string> arm_names_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> enabled_joint_names_;
  std::vector<std::string> ee_names_;
  std::vector<std::string> contact_names_;
  std::vector<std::string> limb_names_;
  std::vector<std::string> dof_names_;
  std::string base_name_;
  std::string imu_name_;
  std::string robot_name_;
  std::string robot_model_name_;
  int floating_base_link_id_{0};
  std::string floating_base_link_name_;
  Eigen::Vector3d fb_origin_offset_{Eigen::Vector3d::Zero()};

  limb_joint_idxs_map_t joint_limb_idx_;
  std::vector<unsigned int> joint_leg_idx_;
  std::vector<unsigned int> joint_arm_idx_;
  joint_idxs_map_t joint_idx_;
  limb_joint_names_map_t joint_leg_names_;
  limb_joint_names_map_t joint_arm_names_;

  Eigen::VectorXd joint_legs_;
  Eigen::VectorXd joint_arms_;

  unsigned int n_legs_{0};
  unsigned int n_arms_{0};

  double base_length_{0.0};
  double base_width_{0.0};

  Eigen::Affine3d world_T_base_{Eigen::Affine3d::Identity()};
  Eigen::Matrix3d world_R_hf_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d world_R_base_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d hf_R_base_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d world_RPY_base_{Eigen::Vector3d::Zero()};
  double base_yaw_{0.0};

  std::map<std::string,Eigen::Vector3d> base_X_foot_;
  std::map<std::string,Eigen::Vector3d> world_X_foot_;
  std::map<std::string,Eigen::Affine3d> base_T_foot_;
  std::map<std::string,Eigen::Affine3d> world_T_foot_;
  std::map<std::string,Eigen::Vector3d> base_X_ee_;
  std::map<std::string,Eigen::Vector3d> world_X_ee_;
  std::map<std::string,Eigen::Affine3d> base_T_ee_;
  std::map<std::string,Eigen::Affine3d> world_T_ee_;

  Eigen::VectorXd q_min_;
  Eigen::VectorXd q_max_;
  Eigen::VectorXd qdot_max_;
  Eigen::VectorXd tau_max_;

  Eigen::VectorXd q_stand_up_;
  Eigen::VectorXd q_stand_down_;
  double stand_up_height_{0.0};
  double stand_down_height_{0.0};
  double current_height_{0.0};
  double previous_height_{0.0};

  mutable RigidBodyDynamics::Model rbdl_model_;

  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  Eigen::VectorXd qddot_;
  Eigen::VectorXd tau_;

  mutable Eigen::MatrixXd tmp_Mi_;
  mutable Eigen::MatrixXd tmp_M_;
  mutable Eigen::MatrixXd tmp_jacobian_;
  mutable Eigen::Matrix3d tmp_matrix3d_;
  mutable Eigen::Vector3d tmp_vector3d_;
  mutable Eigen::Vector3d tmp_vector3d_1_;
  mutable Eigen::Affine3d tmp_affine3d_;
  mutable Eigen::Affine3d tmp_affine3d_1_;
  mutable Eigen::Vector6d tmp_vector6d_;
  mutable Eigen::Vector6d tmp_vector6d_1_;
};

} // namespace wolf_wbid

#endif // WOLF_WBID_QUADRUPED_ROBOT_RBDL_H
