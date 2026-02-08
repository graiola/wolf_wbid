/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef WOLF_WBID_QUADRUPED_ROBOT_H
#define WOLF_WBID_QUADRUPED_ROBOT_H

// STD
#include <memory>
#include <atomic>
#include <map>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Common Eigen aliases (Vector6d, etc.)
#include <wolf_controller_utils/common.h>

// URDF
#include <urdf/model.h>

namespace wolf_wbid
{

class QuadrupedRobot
{

public:

  const std::string CLASS_NAME = "QuadrupedRobot";

  typedef std::shared_ptr<QuadrupedRobot> Ptr;

  typedef std::shared_ptr<const QuadrupedRobot> ConstPtr;

  typedef std::map<std::string,std::vector<std::string>>          limb_joint_names_map_t;
  typedef std::map<std::string,std::vector<unsigned int>>         limb_joint_idxs_map_t;
  typedef std::map<std::string,unsigned int>                      joint_idxs_map_t;

  QuadrupedRobot() = default;
  virtual ~QuadrupedRobot() = default;

  virtual bool update( bool update_position = true,
                       bool update_velocity = true,
                       bool update_desired_acceleration = true ) = 0;

  virtual const std::vector<std::string>& getFootNames() const = 0;
  virtual const std::vector<std::string>& getLegNames() const = 0;
  virtual const std::vector<std::string>& getHipNames() const = 0;
  virtual const std::vector<std::string>& getJointNames() const = 0;
  virtual const std::vector<std::string>& getArmNames() const = 0;
  virtual const std::vector<std::string>& getEndEffectorNames() const = 0;
  virtual const std::vector<std::string>& getContactNames() const = 0;
  virtual const std::vector<std::string>& getLimbNames() const = 0;
  virtual const std::string& getBaseLinkName() const = 0;
  virtual const std::string& getImuSensorName() const = 0;
  virtual const std::string& getRobotName() const = 0;
  virtual const std::string& getRobotModelName() const = 0;
  virtual const std::vector<std::string>& getEnabledJointNames() const = 0;
  virtual Eigen::VectorXd getLegJointValues(const Eigen::VectorXd& joints) = 0;
  virtual Eigen::VectorXd getArmJointValues(const Eigen::VectorXd& joints) = 0;

  virtual const std::vector<unsigned int> &getLimbJointsIds(const std::string& limb_name) = 0;

  virtual const unsigned int& getNumberArms() const = 0;
  virtual const unsigned int& getNumberLegs() const = 0;

  virtual const double& getBaseLength() const = 0;
  virtual const double& getBaseWidth() const = 0;

  virtual bool isRobotFalling(const double& dt) const = 0;

  virtual void getFloatingBasePositionInertia(Eigen::Matrix3d& M) = 0;
  virtual void getFloatingBaseOrientationInertia(Eigen::Matrix3d& M) = 0;
  virtual void getLimbInertia(const std::string& limb_name, Eigen::MatrixXd& M) = 0;
  virtual void getLimbInertiaInverse(const std::string& limb_name, Eigen::MatrixXd& Mi) = 0;

  virtual const Eigen::Matrix3d& getBaseRotationInHf() const = 0;
  virtual const Eigen::Matrix3d& getHfRotationInWorld() const = 0;
  virtual const Eigen::Matrix3d& getBaseRotationInWorld() const = 0;
  virtual const Eigen::Vector3d& getBaseRotationInWorldRPY() const = 0;
  virtual const double& getBaseYawInWorld() const = 0;

  virtual void getCOM(Eigen::Vector3d& p_W) const = 0;
  virtual void getCOMVelocity(Eigen::Vector3d& v_W) const = 0;
  virtual void getCOMJacobian(Eigen::MatrixXd& Jcom) const = 0;
  virtual void getCentroidalMomentum(Eigen::Vector6d& h) const = 0;
  virtual void getCentroidalMomentumMatrix(Eigen::MatrixXd& CMM) const = 0;
  virtual void getCentroidalMomentumMatrix(Eigen::MatrixXd& CMM, Eigen::Vector6d& CMMdotQdot) const = 0;
  virtual bool getVelocityTwist(const std::string& link, Eigen::Vector6d& twist) const = 0;
  virtual bool getVelocityTwist(const std::string& distal, const std::string& base, Eigen::Vector6d& twist) const = 0;
  virtual bool computeRelativeJdotQdot(const std::string& target_link, const std::string& base_link, Eigen::Vector6d& jdotqdot) const = 0;
  virtual bool computeJdotQdot(const std::string& link, const Eigen::Vector3d& point, Eigen::Vector6d& jdotqdot) const = 0;
  virtual bool getPose(const std::string& source_frame, Eigen::Affine3d& pose) const = 0;
  virtual bool getPose(const std::string& source_frame, const std::string& target_frame, Eigen::Affine3d& pose) const = 0;
  virtual bool getJacobian(const std::string& link_name, Eigen::MatrixXd& J) const = 0;
  virtual bool getJacobian(const std::string& link_name, const std::string& target_frame, Eigen::MatrixXd& J) const = 0;

  virtual const std::map<std::string, Eigen::Vector3d>& getFeetPositionInWorld() const = 0;
  virtual const std::map<std::string, Eigen::Vector3d>& getFeetPositionInBase() const = 0;
  virtual const std::map<std::string, Eigen::Affine3d>& getFeetPoseInWorld() const = 0;
  virtual const std::map<std::string, Eigen::Affine3d>& getFeetPoseInBase() const = 0;

  virtual Eigen::Vector3d& getFootPositionInWorld(const std::string &name) = 0;
  virtual Eigen::Vector3d& getFootPositionInBase(const std::string &name) = 0;
  virtual Eigen::Affine3d& getFootPoseInWorld(const std::string &name) = 0;
  virtual Eigen::Affine3d& getFootPoseInBase(const std::string &name) = 0;

  virtual const std::map<std::string, Eigen::Vector3d>& getEndEffectorsPositionInWorld() const = 0;
  virtual const std::map<std::string, Eigen::Vector3d>& getEndEffectorsPositionInBase() const = 0;
  virtual const std::map<std::string, Eigen::Affine3d>& getEndEffectorsPoseInWorld() const = 0;
  virtual const std::map<std::string, Eigen::Affine3d>& getEndEffectorsPoseInBase() const = 0;

  virtual Eigen::Vector3d& getEndEffectorPositionInWorld(const std::string &name) = 0;
  virtual Eigen::Vector3d& getEndEffectorPositionInBase(const std::string &name) = 0;
  virtual Eigen::Affine3d& getEndEffectorPoseInWorld(const std::string &name) = 0;
  virtual Eigen::Affine3d& getEndEffectorPoseInBase(const std::string &name) = 0;

  virtual const Eigen::Affine3d& getBasePoseInWorld() const = 0; // This is the floating base pose w.r.t world

  /**
       * @brief check if the joint velocities are above a max value
       * @param qdot input vector to check
       * @return true is the limits are violated
       */
  virtual std::vector<bool> checkJointVelocities(Eigen::VectorXd &qdot) = 0;

  /**
       * @brief check if the joint velocities are above a max value, saturate the value if the limits are violated
       * @param qdot input vector to clamp
       * @return true if the limits are violated
       */
  virtual bool clampJointVelocities(Eigen::VectorXd &qdot) = 0;

  /**
       * @brief check if the joint positions are between a max and min value, saturate the value if the limits are violated
       * @param q input vector to clamp
       * @return true if the limits are violated
       */
  virtual bool clampJointPositions(Eigen::VectorXd &q) = 0;

  /**
       * @brief check if the joint efforts are above a max value, saturate the value if the limits are violated
       * @param tau input vector to clamp
       * @return true if the limits are violated
       */
  virtual bool clampJointEfforts(Eigen::VectorXd &tau) = 0;

  /**
   * @brief get robot's home position when standing up
   * @return q_stand_up
   */
  virtual const Eigen::VectorXd& getStandUpJointPostion() = 0;

  /*
   * @brief get robot's home position when standing down
   * @return q_stand_down
   */
  virtual const Eigen::VectorXd& getStandDownJointPostion() = 0;

  /**
   * @brief get robot's base height when standing up
   * @return height
   */
  virtual const double& getStandUpHeight() = 0;

  /**
   * @brief get robot's base height when standing down
   * @return height
   */
  virtual const double& getStandDownHeight() = 0;

  /**
  * @brief Computes the pose of the source_frame w.r.t. the target_frame
  *
  * @param q The joint positions.
  * @param source_frame The source link name.
  * @param target_frame The target link name.
  * @param pose A homogeneous transformation which transforms a point from source frame to target frame
  *       P_target = T * P_source
  * @return True if both source_frame and target_frame are valid. False otherwise.
  */
  virtual bool getPose(const Eigen::VectorXd& q, const std::string& source_frame, const std::string& target_frame, Eigen::Affine3d& pose) = 0;

  /**
  * @brief Computes the pose of the source_frame w.r.t. the world frame
  *
  * @param q The joint positions.
  * @param source_frame The source link name.
  * @param pose A homogeneous transformation which transforms a point from source frame to world frame
  *       P_world = T * P_source
  * @return True if source_frame is valid. False otherwise.
  */
  virtual bool getPose(const Eigen::VectorXd& q, const std::string& source_frame, Eigen::Affine3d& pose) = 0;


    /**
  * @brief TODO
  */
  virtual bool getRelativeJacobian(const Eigen::VectorXd& q, const std::string& target_link_name, const std::string& base_link_name, Eigen::MatrixXd& J) = 0;

  /**
  * @brief Computes the twist of the source_frame w.r.t. the world frame
  *
  * @param q The joint positions.
  * @param qd The joint velocities.
  * @param source_frame The source link name.
  * @param twist the calculated twist
  * @return True if source_frame is valid. False otherwise.
  */
  virtual bool getTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const std::string& source_frame, Eigen::Vector6d& twist) = 0;

 /**
  * @brief Computes the twist of the distal frame w.r.t. the base frame 
  *
  * @param q The joint positions.
  * @param qd The joint velocities.
  * @param distal The distal link name.
  * @param base The base link name
  * @param twist_rel the calculated relative twist
  * @return True if source_frame is valid. False otherwise.
  */
  virtual bool getTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const std::string& distal, const std::string& base, Eigen::Vector6d& twist_rel) = 0;

  /**
   * @brief Gets the Jacobian of link_name expressed in the world frame, i.e a matrix such that its product with
   * the derivative of the configuration vector gives the velocity twist of link_name (i.e. first linear then angular velocity). The reference point is the origin of the link with link_name name.
   *
   * @param q The joint positions.
   * @param link_name The link name
   * @param J  The Jacobian expressed in the world frame
   * @return True if the link_name and target_frame are valid link names. False otherwise.
   */
  virtual bool getJacobian(const Eigen::VectorXd& q, const std::string& link_name, Eigen::MatrixXd& J) = 0;

  /**
   * @brief Gets the Jacobian of link_name expressed in the target_frame, i.e a matrix such that its product with
   * the derivative of the configuration vector gives the velocity twist of link_name according to target_frame
   * (i.e. first linear then angular velocity).
   * The reference point is the origin of the link with link_name name.
   *
   * @param q The joint positions.
   * @param link_name The link name
   * @param target_frame The target frame name
   * @param J  The Jacobian expressed in the world frame
   * @return True if the link_name is a valid link name. False otherwise.
   */
  virtual bool getJacobian(const Eigen::VectorXd& q, const std::string& link_name, const std::string& target_frame, Eigen::MatrixXd& J) = 0;

  /**
   * @brief Gets the inertia matrix in joint position q
   *
   * @param q The joint positions.
   * @param M The inertia matrix
   */
  //virtual void getInertiaMatrix(const Eigen::VectorXd& q, Eigen::MatrixXd& M) const;


  /**
   * @brief Gets the current robot's height
   *
   * @return robot's height
   */
  virtual double getCurrentHeight() = 0;

  // Low-level model state and dynamics
  virtual int getJointNum() const = 0;
  virtual double getMass() const = 0;
  virtual int getJointIndex(const std::string& joint_name) const = 0;
  virtual bool getJointPosition(Eigen::VectorXd& q) const = 0;
  virtual bool getJointVelocity(Eigen::VectorXd& qd) const = 0;
  virtual bool getJointEffort(Eigen::VectorXd& tau) const = 0;
  virtual const Eigen::VectorXd& getJointPositions() const = 0;
  virtual const Eigen::VectorXd& getJointVelocities() const = 0;
  virtual const Eigen::VectorXd& getJointEfforts() const = 0;
  virtual bool setJointPosition(const Eigen::VectorXd& q) = 0;
  virtual bool setJointVelocity(const Eigen::VectorXd& qd) = 0;
  virtual bool setJointEffort(const Eigen::VectorXd& tau) = 0;
  virtual bool setJointAcceleration(const Eigen::VectorXd& qddot) = 0;
  virtual bool setJointPosition(int i, double q) = 0;
  virtual bool setJointVelocity(int i, double qd) = 0;
  virtual bool setJointEffort(int i, double tau) = 0;

  virtual void computeInverseDynamics(Eigen::VectorXd& tau) const = 0;
  virtual bool isFloatingBase() const = 0;

  virtual bool getFloatingBasePose(Eigen::Affine3d& pose) const = 0;
  virtual bool setFloatingBasePose(const Eigen::Affine3d& pose) = 0;
  virtual bool setFloatingBaseOrientation(const Eigen::Matrix3d& world_R_base) = 0;
  virtual bool setFloatingBaseAngularVelocity(const Eigen::Vector3d& w) = 0;
  virtual bool setFloatingBaseState(const Eigen::Affine3d& pose, const Eigen::Vector6d& twist) = 0;

  virtual void getInertiaMatrix(Eigen::MatrixXd& M) const = 0;
  virtual void getInertiaInverse(Eigen::MatrixXd& Minv) const = 0;
  virtual void computeNonlinearTerm(Eigen::VectorXd& h) const = 0;
  virtual void computeGravityCompensation(Eigen::VectorXd& g) const = 0;

  virtual bool getOrientation(const std::string& frame, Eigen::Matrix3d& R) const = 0;
  virtual bool getOrientation(const std::string& source_frame, const std::string& target_frame, Eigen::Matrix3d& R) const = 0;

  virtual const urdf::ModelInterface& getUrdf() const = 0;
  virtual const std::string& getUrdfString() const = 0;
  virtual const std::string& getSrdfString() const = 0;
  virtual bool getJointLimits(Eigen::VectorXd& qmin, Eigen::VectorXd& qmax) const = 0;
  virtual bool getVelocityLimits(Eigen::VectorXd& qdot_max) const = 0;
  virtual bool getEffortLimits(Eigen::VectorXd& tau_max) const = 0;

};

} //@namespace wolf_controller

#endif //QUADRUPED_ROBOT_H
