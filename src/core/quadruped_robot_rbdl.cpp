/**
 * @file quadruped_robot_rbdl.cpp
 * @brief Quadruped robot model (RBDL backend).
 */

#include <wolf_wbid/core/quadruped_robot_rbdl.h>
#include <wolf_controller_utils/geometry.h>
#include <wolf_controller_utils/common.h>
#include <stdexcept>
#include <algorithm>
#include <unordered_set>
#include <limits>

namespace wolf_wbid {

static inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d S;
  S <<     0.0, -v.z(),  v.y(),
        v.z(),     0.0, -v.x(),
       -v.y(),  v.x(),    0.0;
  return S;
}

static inline Eigen::Affine3d urdfPoseToEigen(const urdf::Pose& pose)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  double r, p, y;
  pose.rotation.getRPY(r, p, y);
  T.linear() = (Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  return T;
}

QuadrupedRobotRBDL::QuadrupedRobotRBDL(const std::string& robot_name,
                                       const std::string& urdf,
                                       const std::string& srdf)
{
  if(!urdf_model_.initString(urdf)) {
    throw std::runtime_error("QuadrupedRobotRBDL: unable to parse URDF string");
  }
  urdf_string_ = urdf;
  srdf_string_ = srdf;

  if(!srdf_model_.initString(urdf_model_, srdf)) {
    throw std::runtime_error("QuadrupedRobotRBDL: unable to parse SRDF string");
  }

  rbdl_model_ = RigidBodyDynamics::Model();
  if(!RigidBodyDynamics::Addons::URDFReadFromString(urdf.c_str(), &rbdl_model_, true, false)) {
    throw std::runtime_error("QuadrupedRobotRBDL: cannot initialize RBDL model");
  }

  parser_.parseSRDF(urdf, srdf);
  robot_model_name_ = parser_.getRobotModelName();
  leg_names_        = parser_.getLegNames();
  foot_names_       = parser_.getFootNames();
  joint_leg_names_  = parser_.getJointLegNames();
  arm_names_        = parser_.getArmNames();
  ee_names_         = parser_.getEndEffectorNames();
  joint_arm_names_  = parser_.getJointArmNames();
  hip_names_        = parser_.getHipNames();
  base_name_        = parser_.getBaseLinkName();
  imu_name_         = parser_.getImuLinkName();
  robot_name_       = robot_name;

  if(rbdl_model_.q_size >= FLOATING_BASE_DOFS && rbdl_model_.mBodies.size() > 2) {
    floating_base_link_id_ = 2;
    floating_base_link_name_ = rbdl_model_.GetBodyName(floating_base_link_id_);
  } else {
    floating_base_link_id_ = linkId(base_name_);
    floating_base_link_name_ = base_name_;
    if(floating_base_link_id_ < 0) {
      floating_base_link_id_ = 0;
    }
  }
  Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(rbdl_model_.q_size);
  fb_origin_offset_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(
        rbdl_model_, q_zero, floating_base_link_id_, Eigen::Vector3d::Zero(), true);

  n_legs_ = leg_names_.size();
  n_arms_ = arm_names_.size();

  if(n_legs_ != N_LEGS) {
    throw std::runtime_error("QuadrupedRobotRBDL: wrong number of legs in SRDF");
  }
  if(n_arms_ > N_ARMS) {
    throw std::runtime_error("QuadrupedRobotRBDL: wrong number of arms in SRDF");
  }
  if(foot_names_.size() != N_LEGS) {
    throw std::runtime_error("QuadrupedRobotRBDL: wrong number of feet in SRDF");
  }
  if(hip_names_.size() != N_LEGS) {
    throw std::runtime_error("QuadrupedRobotRBDL: wrong number of hips in SRDF");
  }
  if(base_name_.empty()) {
    throw std::runtime_error("QuadrupedRobotRBDL: base link is empty in SRDF");
  }

  // DoF names: floating base + joint order
  dof_names_.clear();
  dof_names_.push_back("base_x");
  dof_names_.push_back("base_y");
  dof_names_.push_back("base_z");
  dof_names_.push_back("base_roll");
  dof_names_.push_back("base_pitch");
  dof_names_.push_back("base_yaw");

  joint_names_.clear();
  joint_idx_.clear();

  // Build joint index map in RBDL order
  for(const auto& joint : urdf_model_.joints_) {
    if(joint.second->type == urdf::Joint::FIXED) continue;
    const auto& jname = joint.first;
    auto j = urdf_model_.getJoint(jname);
    if(!j) continue;
    const std::string& child = j->child_link_name;
    int body_id = linkId(child);
    if(body_id < 0) continue;
    int q_index = rbdl_model_.mJoints[body_id].q_index;
    joint_idx_[jname] = q_index;
  }

  // Joint names ordered by q_index (actuated joints only)
  std::unordered_set<std::string> allowed_joints;
  for(const auto& kv : joint_leg_names_) {
    for(const auto& jn : kv.second) allowed_joints.insert(jn);
  }
  for(const auto& kv : joint_arm_names_) {
    for(const auto& jn : kv.second) allowed_joints.insert(jn);
  }

  std::vector<std::pair<int,std::string>> ordered;
  ordered.reserve(joint_idx_.size());
  for(const auto& kv : joint_idx_) {
    if(allowed_joints.find(kv.first) == allowed_joints.end()) {
      continue;
    }
    ordered.emplace_back(kv.second, kv.first);
  }
  std::sort(ordered.begin(), ordered.end(),
            [](const auto& a, const auto& b){ return a.first < b.first; });
  for(const auto& kv : ordered) {
    joint_names_.push_back(kv.second);
  }

  // Expand dof_names_ with joints in q order
  std::vector<std::string> dof_names_joint(rbdl_model_.q_size - FLOATING_BASE_DOFS, "");
  for(const auto& kv : joint_idx_) {
    int qi = kv.second - FLOATING_BASE_DOFS;
    if(qi >= 0 && qi < static_cast<int>(dof_names_joint.size())) {
      dof_names_joint[qi] = kv.first;
    }
  }
  for(const auto& name : dof_names_joint) {
    if(!name.empty()) {
      dof_names_.push_back(name);
    }
  }
  enabled_joint_names_ = dof_names_;

  // Build limb joint indices
  for(const auto& lname : leg_names_) {
    for(const auto& jn : joint_leg_names_[lname]) {
      int idx = joint_idx_.at(jn);
      joint_limb_idx_[lname].push_back(idx);
      joint_leg_idx_.push_back(idx);
    }
    std::sort(joint_limb_idx_[lname].begin(), joint_limb_idx_[lname].end());
  }

  for(const auto& aname : arm_names_) {
    for(const auto& jn : joint_arm_names_[aname]) {
      int idx = joint_idx_.at(jn);
      joint_limb_idx_[aname].push_back(idx);
      joint_arm_idx_.push_back(idx);
    }
    std::sort(joint_limb_idx_[aname].begin(), joint_limb_idx_[aname].end());
  }

  limb_names_.clear();
  limb_names_.insert(limb_names_.end(), leg_names_.begin(), leg_names_.end());
  limb_names_.insert(limb_names_.end(), arm_names_.begin(), arm_names_.end());

  contact_names_ = foot_names_;
  contact_names_.insert(contact_names_.end(), ee_names_.begin(), ee_names_.end());

  // Allocate state
  q_.setZero(rbdl_model_.q_size);
  qdot_.setZero(rbdl_model_.qdot_size);
  qddot_.setZero(rbdl_model_.qdot_size);
  tau_.setZero(rbdl_model_.qdot_size);

  // Initialize transforms
  world_T_base_.setIdentity();
  world_R_hf_.setIdentity();
  world_R_base_.setIdentity();
  hf_R_base_.setIdentity();
  world_RPY_base_.setZero();

  for(const auto& fn : foot_names_) {
    world_X_foot_[fn] = Eigen::Vector3d::Zero();
    base_X_foot_[fn] = Eigen::Vector3d::Zero();
    world_T_foot_[fn] = Eigen::Affine3d::Identity();
    base_T_foot_[fn] = Eigen::Affine3d::Identity();
  }
  for(const auto& ee : ee_names_) {
    world_X_ee_[ee] = Eigen::Vector3d::Zero();
    base_X_ee_[ee] = Eigen::Vector3d::Zero();
    world_T_ee_[ee] = Eigen::Affine3d::Identity();
    base_T_ee_[ee] = Eigen::Affine3d::Identity();
  }

  // Base size from hip positions
  Eigen::Affine3d pose_lf, pose_lh, pose_rf, pose_rh;
  getPose(hip_names_[0], base_name_, pose_lf);
  getPose(hip_names_[1], base_name_, pose_lh);
  getPose(hip_names_[2], base_name_, pose_rf);
  getPose(hip_names_[3], base_name_, pose_rh);
  base_width_  = std::abs(pose_lf.translation().y() - pose_rf.translation().y());
  base_length_ = std::abs(pose_lf.translation().x() - pose_lh.translation().x());

  // Limits
  getEffortLimits(tau_max_);
  getVelocityLimits(qdot_max_);
  getJointLimits(q_min_, q_max_);
  if(tau_max_.size() >= FLOATING_BASE_DOFS) {
    tau_max_.head(FLOATING_BASE_DOFS).setZero();
  }

  // Home positions
  if(!loadSrdfState("standup", q_stand_up_)) {
    throw std::runtime_error("QuadrupedRobotRBDL: standup state missing in SRDF");
  }
  if(!loadSrdfState("standdown", q_stand_down_)) {
    throw std::runtime_error("QuadrupedRobotRBDL: standdown state missing in SRDF");
  }

  // Stand-up/down heights
  stand_up_height_ = 0.0;
  Eigen::Affine3d pose;
  for(const auto& fn : foot_names_) {
    getPose(q_stand_up_, fn, base_name_, pose);
    stand_up_height_ += pose.translation().z();
  }
  stand_up_height_ = -stand_up_height_ / N_LEGS;

  stand_down_height_ = 0.0;
  for(const auto& fn : foot_names_) {
    getPose(q_stand_down_, fn, base_name_, pose);
    stand_down_height_ += pose.translation().z();
  }
  stand_down_height_ = -stand_down_height_ / N_LEGS;
}

int QuadrupedRobotRBDL::linkId(const std::string& link_name) const
{
  if(link_name == "world") {
    return 0; // RBDL root
  }
  unsigned int id = rbdl_model_.GetBodyId(link_name.c_str());
  if(id == std::numeric_limits<unsigned int>::max()) {
    // Allow passing a joint name: resolve to its child link.
    auto joint = urdf_model_.getJoint(link_name);
    if(joint) {
      id = rbdl_model_.GetBodyId(joint->child_link_name.c_str());
    }
    if(id == std::numeric_limits<unsigned int>::max()) {
      return -1;
    }
  }
  return static_cast<int>(id);
}

bool QuadrupedRobotRBDL::resolveFixedLink(const std::string& link_name,
                                          std::string& ancestor_link,
                                          Eigen::Affine3d& T_ancestor_to_link) const
{
  T_ancestor_to_link.setIdentity();
  std::string current = link_name;
  for(int depth = 0; depth < 64; ++depth) {
    if(linkId(current) >= 0) {
      ancestor_link = current;
      return true;
    }
    auto urdf_link = urdf_model_.getLink(current);
    if(!urdf_link || !urdf_link->parent_joint) {
      return false;
    }
    auto joint = urdf_link->parent_joint;
    if(joint->type != urdf::Joint::FIXED) {
      return false;
    }
    Eigen::Affine3d T_parent_joint = urdfPoseToEigen(joint->parent_to_joint_origin_transform);
    T_ancestor_to_link = T_parent_joint * T_ancestor_to_link;
    current = joint->parent_link_name;
  }
  return false;
}

bool QuadrupedRobotRBDL::loadSrdfState(const std::string& state_name, Eigen::VectorXd& q) const
{
  const auto& states = srdf_model_.getGroupStates();
  auto it = std::find_if(states.begin(), states.end(),
                         [&](const srdf::Model::GroupState& s){ return s.name_ == state_name; });
  if(it == states.end()) return false;

  q.setZero(rbdl_model_.q_size);
  for(const auto& jp : it->joint_values_) {
    auto map_it = joint_idx_.find(jp.first);
    if(map_it != joint_idx_.end()) {
      q(map_it->second) = jp.second[0];
    }
  }
  return true;
}

bool QuadrupedRobotRBDL::update(bool update_position, bool update_velocity, bool update_desired_acceleration)
{
  (void)update_position;
  (void)update_velocity;
  (void)update_desired_acceleration;

  RigidBodyDynamics::UpdateKinematics(rbdl_model_, q_, qdot_, qddot_);

  // Update base transform
  getPose(base_name_, world_T_base_);
  world_R_hf_ = Eigen::Matrix3d::Identity();
  base_yaw_   = std::atan2(world_T_base_.linear()(1,0), world_T_base_.linear()(0,0));
  world_R_hf_ = Eigen::AngleAxisd(base_yaw_, Eigen::Vector3d::UnitZ());
  world_R_base_ = world_T_base_.linear();
  wolf_controller_utils::rotToRpy(world_R_base_, world_RPY_base_);
  hf_R_base_ = world_R_hf_.transpose() * world_R_base_;

  // Feet
  for(const auto& fn : foot_names_) {
    getPose(fn, world_T_foot_[fn]);
    world_X_foot_[fn] = world_T_foot_[fn].translation();
    getPose(fn, base_name_, base_T_foot_[fn]);
    base_X_foot_[fn] = base_T_foot_[fn].translation();
  }

  // End-effectors
  for(const auto& ee : ee_names_) {
    getPose(ee, world_T_ee_[ee]);
    world_X_ee_[ee] = world_T_ee_[ee].translation();
    getPose(ee, base_name_, base_T_ee_[ee]);
    base_X_ee_[ee] = base_T_ee_[ee].translation();
  }

  previous_height_ = current_height_;
  getFloatingBasePose(tmp_affine3d_);
  current_height_ = tmp_affine3d_.translation().z();

  return true;
}

const std::vector<std::string>& QuadrupedRobotRBDL::getFootNames() const { return foot_names_; }
const std::vector<std::string>& QuadrupedRobotRBDL::getLegNames() const { return leg_names_; }
const std::vector<std::string>& QuadrupedRobotRBDL::getHipNames() const { return hip_names_; }
const std::vector<std::string>& QuadrupedRobotRBDL::getJointNames() const { return joint_names_; }
const std::vector<std::string>& QuadrupedRobotRBDL::getArmNames() const { return arm_names_; }
const std::vector<std::string>& QuadrupedRobotRBDL::getEndEffectorNames() const { return ee_names_; }
const std::vector<std::string>& QuadrupedRobotRBDL::getContactNames() const { return contact_names_; }
const std::vector<std::string>& QuadrupedRobotRBDL::getLimbNames() const { return limb_names_; }
const std::string& QuadrupedRobotRBDL::getBaseLinkName() const { return base_name_; }
const std::string& QuadrupedRobotRBDL::getImuSensorName() const { return imu_name_; }
const std::string& QuadrupedRobotRBDL::getRobotName() const { return robot_name_; }
const std::string& QuadrupedRobotRBDL::getRobotModelName() const { return robot_model_name_; }
const std::vector<std::string>& QuadrupedRobotRBDL::getEnabledJointNames() const { return enabled_joint_names_; }

Eigen::VectorXd QuadrupedRobotRBDL::getLegJointValues(const Eigen::VectorXd& joints)
{
  Eigen::VectorXd joints_out(joint_leg_idx_.size() + FLOATING_BASE_DOFS);
  joints_out.head(FLOATING_BASE_DOFS) = joints.head(FLOATING_BASE_DOFS);
  for(unsigned int i = 0; i < joint_leg_idx_.size(); ++i) {
    joints_out(FLOATING_BASE_DOFS + i) = joints(joint_leg_idx_[i]);
  }
  return joints_out;
}

Eigen::VectorXd QuadrupedRobotRBDL::getArmJointValues(const Eigen::VectorXd& joints)
{
  Eigen::VectorXd joints_out(joint_arm_idx_.size() + FLOATING_BASE_DOFS);
  joints_out.head(FLOATING_BASE_DOFS) = joints.head(FLOATING_BASE_DOFS);
  for(unsigned int i = 0; i < joint_arm_idx_.size(); ++i) {
    joints_out(FLOATING_BASE_DOFS + i) = joints(joint_arm_idx_[i]);
  }
  return joints_out;
}

const std::vector<unsigned int>& QuadrupedRobotRBDL::getLimbJointsIds(const std::string& limb_name)
{
  return joint_limb_idx_[limb_name];
}

const unsigned int& QuadrupedRobotRBDL::getNumberArms() const { return n_arms_; }
const unsigned int& QuadrupedRobotRBDL::getNumberLegs() const { return n_legs_; }
const double& QuadrupedRobotRBDL::getBaseLength() const { return base_length_; }
const double& QuadrupedRobotRBDL::getBaseWidth() const { return base_width_; }

bool QuadrupedRobotRBDL::isRobotFalling(const double& dt) const
{
  return (((current_height_ - previous_height_) / dt <= EPS) ? false : true);
}

void QuadrupedRobotRBDL::getFloatingBasePositionInertia(Eigen::Matrix3d& M)
{
  getInertiaMatrix(tmp_M_);
  M = tmp_M_.block(0,0,3,3);
}

void QuadrupedRobotRBDL::getFloatingBaseOrientationInertia(Eigen::Matrix3d& M)
{
  getInertiaMatrix(tmp_M_);
  M = tmp_M_.block(3,3,3,3);
}

void QuadrupedRobotRBDL::getLimbInertia(const std::string& limb_name, Eigen::MatrixXd& M)
{
  getInertiaMatrix(tmp_M_);
  const auto& idxs = joint_limb_idx_[limb_name];
  int n = static_cast<int>(idxs.size());
  if(n == 0) {
    M.resize(0,0);
    return;
  }
  int idx = idxs.front();
  if(idx + n <= tmp_M_.rows()) {
    M = tmp_M_.block(idx,idx,n,n);
    return;
  }
  M.setZero(n,n);
  for(int i = 0; i < n; ++i) {
    for(int j = 0; j < n; ++j) {
      M(i,j) = tmp_M_(idxs[i], idxs[j]);
    }
  }
}

void QuadrupedRobotRBDL::getLimbInertiaInverse(const std::string& limb_name, Eigen::MatrixXd& Mi)
{
  getInertiaMatrix(tmp_M_);
  if(tmp_M_.rows() == 0 || tmp_M_.cols() == 0 || tmp_M_.rows() != tmp_M_.cols()) {
    Mi.resize(0,0);
    return;
  }
  if(tmp_M_.rows() < FLOATING_BASE_DOFS) {
    Mi.resize(0,0);
    return;
  }
  tmp_Mi_.setZero();
  tmp_Mi_.block(FLOATING_BASE_DOFS,FLOATING_BASE_DOFS,tmp_M_.rows()-FLOATING_BASE_DOFS,tmp_M_.cols()-FLOATING_BASE_DOFS)
      = tmp_M_.block(FLOATING_BASE_DOFS,FLOATING_BASE_DOFS,tmp_M_.rows()-FLOATING_BASE_DOFS,tmp_M_.cols()-FLOATING_BASE_DOFS).inverse();
  const auto& idxs = joint_limb_idx_[limb_name];
  int n = static_cast<int>(idxs.size());
  if(n == 0) {
    Mi.resize(0,0);
    return;
  }
  int idx = idxs.front();
  if(idx + n <= tmp_Mi_.rows()) {
    Mi = tmp_Mi_.block(idx,idx,n,n);
    return;
  }
  Mi.setZero(n,n);
  for(int i = 0; i < n; ++i) {
    for(int j = 0; j < n; ++j) {
      Mi(i,j) = tmp_Mi_(idxs[i], idxs[j]);
    }
  }
}

const Eigen::Matrix3d& QuadrupedRobotRBDL::getBaseRotationInHf() const { return hf_R_base_; }
const Eigen::Matrix3d& QuadrupedRobotRBDL::getHfRotationInWorld() const { return world_R_hf_; }
const Eigen::Matrix3d& QuadrupedRobotRBDL::getBaseRotationInWorld() const { return world_R_base_; }
const Eigen::Vector3d& QuadrupedRobotRBDL::getBaseRotationInWorldRPY() const { return world_RPY_base_; }
const double& QuadrupedRobotRBDL::getBaseYawInWorld() const { return base_yaw_; }

void QuadrupedRobotRBDL::getCOM(Eigen::Vector3d& p_W) const
{
  double mass;
  RigidBodyDynamics::Math::Vector3d com;
  RigidBodyDynamics::Utils::CalcCenterOfMass(rbdl_model_, q_, qdot_, mass, com, nullptr, nullptr, false);
  p_W = com;
}

void QuadrupedRobotRBDL::getCOMVelocity(Eigen::Vector3d& v_W) const
{
  double mass;
  RigidBodyDynamics::Math::Vector3d com;
  RigidBodyDynamics::Math::Vector3d v_com;
  RigidBodyDynamics::Utils::CalcCenterOfMass(rbdl_model_, q_, qdot_, mass, com, &v_com, nullptr, true);
  v_W = v_com;
}

void QuadrupedRobotRBDL::getCOMJacobian(Eigen::MatrixXd& Jcom) const
{
  tmp_jacobian_.setZero(6, rbdl_model_.dof_count);
  double mass = 0.0;
  int body_id = 0;

  for(body_id = 1; body_id < static_cast<int>(rbdl_model_.mBodies.size()); ++body_id) {
    const RigidBodyDynamics::Body& body = rbdl_model_.mBodies[body_id];
    Eigen::MatrixXd Jtmp(3, rbdl_model_.dof_count);
    Jtmp.setZero();
    RigidBodyDynamics::CalcPointJacobian(rbdl_model_, q_, body_id, body.mCenterOfMass, Jtmp, false);
    tmp_jacobian_.block(0,0,3,rbdl_model_.dof_count) += body.mMass * Jtmp;
    mass += body.mMass;
  }

  tmp_jacobian_ /= mass;
  Jcom = tmp_jacobian_.topRows(3);
}

void QuadrupedRobotRBDL::getCentroidalMomentum(Eigen::Vector6d& h) const
{
  double mass;
  RigidBodyDynamics::Math::Vector3d tmp_com;
  RigidBodyDynamics::Math::Vector3d tmp_lin;
  RigidBodyDynamics::Math::Vector3d tmp_ang;
  RigidBodyDynamics::Utils::CalcCenterOfMass(rbdl_model_, q_, qdot_, mass, tmp_com, &tmp_lin, &tmp_ang, false);
  h.head<3>() = tmp_lin * mass;
  h.tail<3>() = tmp_ang;
}

void QuadrupedRobotRBDL::getCentroidalMomentumMatrix(Eigen::MatrixXd& CMM) const
{
  Eigen::Vector6d cmmdotqdot;
  getCentroidalMomentumMatrix(CMM, cmmdotqdot);
}

void QuadrupedRobotRBDL::getCentroidalMomentumMatrix(Eigen::MatrixXd& CMM, Eigen::Vector6d& CMMdotQdot) const
{
  CMM.setZero(6, getJointNum());
  if(getJointNum() < FLOATING_BASE_DOFS) {
    return;
  }

  getInertiaMatrix(tmp_M_);
  Eigen::Vector3d com;
  getCOM(com);

  Eigen::Affine3d w_T_fb;
  getFloatingBasePose(w_T_fb);
  Eigen::Vector3d fb_com = w_T_fb.inverse() * com;

  Eigen::MatrixXd Jfb;
  getJacobian(base_name_, fb_com, Jfb);

  Eigen::Matrix<double, 6, 6> Ju = Jfb.block<6,6>(0,0);
  Eigen::Matrix<double, 6, 6> Ju_T_inv = Ju.transpose().inverse();

  CMM.noalias() = Ju_T_inv * tmp_M_.block(0,0,6,getJointNum());

  Eigen::VectorXd h;
  Eigen::VectorXd gcomp;
  computeNonlinearTerm(h);
  computeGravityCompensation(gcomp);
  CMMdotQdot.noalias() = Ju_T_inv * (h - gcomp).head<6>();
}

bool QuadrupedRobotRBDL::getVelocityTwist(const std::string& link, Eigen::Vector6d& twist) const
{
  int body_id = linkId(link);
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  if(body_id < 0) {
    std::string ancestor;
    Eigen::Affine3d T_ancestor_to_link;
    if(!resolveFixedLink(link, ancestor, T_ancestor_to_link)) {
      return false;
    }
    body_id = linkId(ancestor);
    if(body_id < 0) return false;
    point = T_ancestor_to_link.translation();
  }
  Eigen::Vector6d v = RigidBodyDynamics::CalcPointVelocity6D(rbdl_model_, q_, qdot_, body_id, point, false);
  twist.head<3>() = v.tail<3>();
  twist.tail<3>() = v.head<3>();
  return true;
}

bool QuadrupedRobotRBDL::getVelocityTwist(const std::string& distal, const std::string& base, Eigen::Vector6d& twist) const
{
  if(base == "world") {
    return getVelocityTwist(distal, twist);
  }

  Eigen::MatrixXd J;
  if(!const_cast<QuadrupedRobotRBDL*>(this)->getRelativeJacobian(q_, distal, base, J)) {
    return false;
  }
  twist.noalias() = J * qdot_;
  return true;
}

bool QuadrupedRobotRBDL::computeJdotQdot(const std::string& link, const Eigen::Vector3d& point, Eigen::Vector6d& jdotqdot) const
{
  int body_id = linkId(link);
  Eigen::Vector3d p = point;
  if(body_id < 0) {
    std::string ancestor;
    Eigen::Affine3d T_ancestor_to_link;
    if(!resolveFixedLink(link, ancestor, T_ancestor_to_link)) {
      return false;
    }
    body_id = linkId(ancestor);
    if(body_id < 0) return false;
    p = T_ancestor_to_link.translation();
  }
  Eigen::VectorXd qdd_zero = Eigen::VectorXd::Zero(qdot_.size());
  Eigen::Vector6d acc = RigidBodyDynamics::CalcPointAcceleration6D(rbdl_model_, q_, qdot_, qdd_zero, body_id, p, true);
  jdotqdot.head<3>() = acc.tail<3>();
  jdotqdot.tail<3>() = acc.head<3>();
  return true;
}

bool QuadrupedRobotRBDL::computeRelativeJdotQdot(const std::string& target_link, const std::string& base_link, Eigen::Vector6d& jdotqdot) const
{
  Eigen::Affine3d Tbase, Tlink;
  if(!getPose(base_link, Tbase)) return false;
  if(!getPose(target_link, Tlink)) return false;

  Eigen::Vector6d JdQdbase, JdQdlink;
  Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  if(!computeJdotQdot(base_link, zero, JdQdbase)) return false;
  if(!computeJdotQdot(target_link, zero, JdQdlink)) return false;

  Eigen::Vector6d vbase, vlink;
  if(!getVelocityTwist(base_link, vbase)) return false;
  if(!getVelocityTwist(target_link, vlink)) return false;

  Eigen::Vector3d r = Tlink.translation() - Tbase.translation();

  Eigen::Vector6d JdotQdot;
  JdotQdot.setZero();
  JdotQdot.head(3) = Tbase.linear().transpose() * (JdQdlink.head(3) - JdQdbase.head(3)
                            - JdQdbase.tail<3>().cross(r)
                            - 2.0 * vbase.tail<3>().cross((vlink - vbase).head<3>())
                            + vbase.tail<3>().cross(vbase.tail<3>().cross(r)));
  JdotQdot.tail(3) = Tbase.linear().transpose() * ((JdQdlink.tail(3) - JdQdbase.tail(3)) - (vbase.tail<3>().cross(vlink.tail<3>())));

  jdotqdot = JdotQdot;
  return true;
}

const std::map<std::string, Eigen::Vector3d>& QuadrupedRobotRBDL::getFeetPositionInWorld() const { return world_X_foot_; }
const std::map<std::string, Eigen::Vector3d>& QuadrupedRobotRBDL::getFeetPositionInBase() const { return base_X_foot_; }
const std::map<std::string, Eigen::Affine3d>& QuadrupedRobotRBDL::getFeetPoseInWorld() const { return world_T_foot_; }
const std::map<std::string, Eigen::Affine3d>& QuadrupedRobotRBDL::getFeetPoseInBase() const { return base_T_foot_; }

Eigen::Vector3d& QuadrupedRobotRBDL::getFootPositionInWorld(const std::string &name) { return world_X_foot_[name]; }
Eigen::Vector3d& QuadrupedRobotRBDL::getFootPositionInBase(const std::string &name) { return base_X_foot_[name]; }
Eigen::Affine3d& QuadrupedRobotRBDL::getFootPoseInWorld(const std::string &name) { return world_T_foot_[name]; }
Eigen::Affine3d& QuadrupedRobotRBDL::getFootPoseInBase(const std::string &name) { return base_T_foot_[name]; }

const std::map<std::string, Eigen::Vector3d>& QuadrupedRobotRBDL::getEndEffectorsPositionInWorld() const { return world_X_ee_; }
const std::map<std::string, Eigen::Vector3d>& QuadrupedRobotRBDL::getEndEffectorsPositionInBase() const { return base_X_ee_; }
const std::map<std::string, Eigen::Affine3d>& QuadrupedRobotRBDL::getEndEffectorsPoseInWorld() const { return world_T_ee_; }
const std::map<std::string, Eigen::Affine3d>& QuadrupedRobotRBDL::getEndEffectorsPoseInBase() const { return base_T_ee_; }

Eigen::Vector3d& QuadrupedRobotRBDL::getEndEffectorPositionInWorld(const std::string &name) { return world_X_ee_[name]; }
Eigen::Vector3d& QuadrupedRobotRBDL::getEndEffectorPositionInBase(const std::string &name) { return base_X_ee_[name]; }
Eigen::Affine3d& QuadrupedRobotRBDL::getEndEffectorPoseInWorld(const std::string &name) { return world_T_ee_[name]; }
Eigen::Affine3d& QuadrupedRobotRBDL::getEndEffectorPoseInBase(const std::string &name) { return base_T_ee_[name]; }

const Eigen::Affine3d& QuadrupedRobotRBDL::getBasePoseInWorld() const { return world_T_base_; }

std::vector<bool> QuadrupedRobotRBDL::checkJointVelocities(Eigen::VectorXd &qdot)
{
  std::vector<bool> violated_limits(qdot.size());
  for(unsigned int i=0;i<qdot_max_.size();i++) {
    violated_limits[i] = (std::abs(qdot(i))>qdot_max_(i));
  }
  return violated_limits;
}

bool QuadrupedRobotRBDL::clampJointVelocities(Eigen::VectorXd &qdot)
{
  bool violated_limits = false;
  for(unsigned int i=0;i<qdot_max_.size();i++) {
    if(std::abs(qdot(i))>qdot_max_(i)) {
      qdot(i) = (qdot(i) > 0.0 ? qdot_max_(i) : -qdot_max_(i));
      violated_limits = true;
    }
  }
  return violated_limits;
}

bool QuadrupedRobotRBDL::clampJointPositions(Eigen::VectorXd &q)
{
  bool violated_limits = false;
  for(unsigned int i=0;i<q.size();i++) {
    if(q(i)<q_min_(i)) {
      q(i) = q_min_(i);
      violated_limits = true;
    } else if(q(i)>q_max_(i)) {
      q(i) = q_max_(i);
      violated_limits = true;
    }
  }
  return violated_limits;
}

bool QuadrupedRobotRBDL::clampJointEfforts(Eigen::VectorXd &tau)
{
  bool violated_limits = false;
  for(unsigned int i=0;i<tau.size();i++) {
    if(std::abs(tau(i))>tau_max_(i)+EPS) {
      violated_limits = true;
    }
  }
  return violated_limits;
}

const Eigen::VectorXd& QuadrupedRobotRBDL::getStandUpJointPostion() { return q_stand_up_; }
const Eigen::VectorXd& QuadrupedRobotRBDL::getStandDownJointPostion() { return q_stand_down_; }
const double& QuadrupedRobotRBDL::getStandUpHeight() { return stand_up_height_; }
const double& QuadrupedRobotRBDL::getStandDownHeight() { return stand_down_height_; }

bool QuadrupedRobotRBDL::getPose(const Eigen::VectorXd& q, const std::string& source_frame, Eigen::Affine3d& pose)
{
  if(source_frame == "world") {
    pose = Eigen::Affine3d::Identity();
    return true;
  }
  int body_id = linkId(source_frame);
  if(body_id < 0) {
    std::string ancestor;
    Eigen::Affine3d T_ancestor_to_link;
    if(!resolveFixedLink(source_frame, ancestor, T_ancestor_to_link)) {
      return false;
    }
    Eigen::Affine3d T_ancestor;
    if(!getPose(q, ancestor, T_ancestor)) {
      return false;
    }
    pose = T_ancestor * T_ancestor_to_link;
    return true;
  }
  if(body_id == 0) {
    pose = Eigen::Affine3d::Identity();
    return true;
  }
  tmp_vector3d_.setZero();
  tmp_matrix3d_ = RigidBodyDynamics::CalcBodyWorldOrientation(rbdl_model_, q, body_id, true);
  tmp_vector3d_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl_model_, q, body_id, tmp_vector3d_, true);
  tmp_matrix3d_.transposeInPlace();
  pose.linear() = tmp_matrix3d_;
  pose.translation() = tmp_vector3d_;
  return true;
}

bool QuadrupedRobotRBDL::getPose(const Eigen::VectorXd& q, const std::string& source_frame, const std::string& target_frame, Eigen::Affine3d& pose)
{
  bool ret = true;
  ret = getPose(q, source_frame, tmp_affine3d_) && ret;
  ret = getPose(q, target_frame, tmp_affine3d_1_) && ret;
  if(!ret) return false;
  pose = tmp_affine3d_1_.inverse() * tmp_affine3d_;
  return true;
}

bool QuadrupedRobotRBDL::getJacobian(const Eigen::VectorXd &q, const std::string &link_name, Eigen::MatrixXd &J)
{
  int body_id = linkId(link_name);
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  if(body_id < 0) {
    std::string ancestor;
    Eigen::Affine3d T_ancestor_to_link;
    if(!resolveFixedLink(link_name, ancestor, T_ancestor_to_link)) {
      return false;
    }
    body_id = linkId(ancestor);
    if(body_id < 0) {
      return false;
    }
    point = T_ancestor_to_link.translation();
  }
  tmp_jacobian_.setZero(6, rbdl_model_.dof_count);
  RigidBodyDynamics::CalcPointJacobian6D(rbdl_model_, q, body_id, point, tmp_jacobian_, true);
  J.resize(6, rbdl_model_.dof_count);
  J.topRows(3) = tmp_jacobian_.bottomRows(3);
  J.bottomRows(3) = tmp_jacobian_.topRows(3);
  return true;
}

bool QuadrupedRobotRBDL::getJacobian(const Eigen::VectorXd &q, const std::string &link_name, const std::string &target_frame, Eigen::MatrixXd &J)
{
  if(!getJacobian(q, link_name, J)) return false;
  Eigen::Matrix3d R;
  if(!getOrientation(target_frame, R)) return false;
  J.topRows(3) = R.transpose() * J.topRows(3);
  J.bottomRows(3) = R.transpose() * J.bottomRows(3);
  return true;
}

bool QuadrupedRobotRBDL::getJacobian(const std::string& link_name, const Eigen::Vector3d& point, Eigen::MatrixXd& J) const
{
  int body_id = linkId(link_name);
  if(body_id < 0) return false;
  tmp_jacobian_.setZero(6, rbdl_model_.dof_count);
  RigidBodyDynamics::CalcPointJacobian6D(rbdl_model_, q_, body_id, point, tmp_jacobian_, true);
  J.resize(6, rbdl_model_.dof_count);
  J.topRows(3) = tmp_jacobian_.bottomRows(3);
  J.bottomRows(3) = tmp_jacobian_.topRows(3);
  return true;
}

bool QuadrupedRobotRBDL::getRelativeJacobian(const Eigen::VectorXd& q,
                                             const std::string& target_link_name,
                                             const std::string& base_link_name,
                                             Eigen::MatrixXd& J)
{
  Eigen::MatrixXd J_base_w, J_target_w;
  if(!getJacobian(q, base_link_name, J_base_w)) return false;
  if(!getJacobian(q, target_link_name, J_target_w)) return false;

  Eigen::Affine3d w_T_base, w_T_target;
  if(!getPose(q, base_link_name, w_T_base)) return false;
  if(!getPose(q, target_link_name, w_T_target)) return false;

  const Eigen::Vector3d w_p_base   = w_T_base.translation();
  const Eigen::Vector3d w_p_target = w_T_target.translation();

  Eigen::MatrixXd J_base_shift = J_base_w;
  J_base_shift.topRows(3).noalias() += skew(w_p_target - w_p_base) * J_base_w.bottomRows(3);

  Eigen::MatrixXd J_rel_w = J_target_w - J_base_shift;

  const Eigen::Matrix3d w_R_base = w_T_base.linear();
  J.resize(6, J_rel_w.cols());
  J.topRows(3) = w_R_base.transpose() * J_rel_w.topRows(3);
  J.bottomRows(3) = w_R_base.transpose() * J_rel_w.bottomRows(3);
  return true;
}

bool QuadrupedRobotRBDL::getTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const std::string& source_frame, Eigen::Vector6d& twist)
{
  int body_id = linkId(source_frame);
  if(body_id < 0) return false;
  tmp_vector3d_.setZero();
  Eigen::Vector6d v = RigidBodyDynamics::CalcPointVelocity6D(rbdl_model_, q, qd, body_id, tmp_vector3d_, false);
  twist.head<3>() = v.tail<3>();
  twist.tail<3>() = v.head<3>();
  return true;
}

bool QuadrupedRobotRBDL::getTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const std::string& distal, const std::string& base, Eigen::Vector6d& twist_rel)
{
  if(distal == base) {
    twist_rel.setZero();
    return true;
  }
  Eigen::MatrixXd J;
  if(base == "world") {
    if(!getJacobian(q, distal, "world", J)) return false;
  } else {
    if(!getRelativeJacobian(q, distal, base, J)) return false;
  }
  twist_rel.noalias() = J * qd;
  return true;
}

bool QuadrupedRobotRBDL::getPose(const std::string& source_frame, Eigen::Affine3d& pose) const
{
  return const_cast<QuadrupedRobotRBDL*>(this)->getPose(q_, source_frame, pose);
}

bool QuadrupedRobotRBDL::getPose(const std::string& source_frame, const std::string& target_frame, Eigen::Affine3d& pose) const
{
  return const_cast<QuadrupedRobotRBDL*>(this)->getPose(q_, source_frame, target_frame, pose);
}

bool QuadrupedRobotRBDL::getJacobian(const std::string& link_name, Eigen::MatrixXd& J) const
{
  return const_cast<QuadrupedRobotRBDL*>(this)->getJacobian(q_, link_name, J);
}

bool QuadrupedRobotRBDL::getJacobian(const std::string& link_name, const std::string& target_frame, Eigen::MatrixXd& J) const
{
  return const_cast<QuadrupedRobotRBDL*>(this)->getJacobian(q_, link_name, target_frame, J);
}

double QuadrupedRobotRBDL::getCurrentHeight() { return current_height_; }

int QuadrupedRobotRBDL::getJointNum() const { return static_cast<int>(q_.size()); }
double QuadrupedRobotRBDL::getMass() const
{
  double mass = 0.0;
  for(const auto& body : rbdl_model_.mBodies) {
    mass += body.mMass;
  }
  return mass;
}

int QuadrupedRobotRBDL::getJointIndex(const std::string& joint_name) const
{
  auto it = joint_idx_.find(joint_name);
  if(it == joint_idx_.end()) return -1;
  return it->second;
}
bool QuadrupedRobotRBDL::getJointPosition(Eigen::VectorXd& q) const { q = q_; return true; }
bool QuadrupedRobotRBDL::getJointVelocity(Eigen::VectorXd& qd) const { qd = qdot_; return true; }
bool QuadrupedRobotRBDL::getJointEffort(Eigen::VectorXd& tau) const { tau = tau_; return true; }
const Eigen::VectorXd& QuadrupedRobotRBDL::getJointPositions() const { return q_; }
const Eigen::VectorXd& QuadrupedRobotRBDL::getJointVelocities() const { return qdot_; }
const Eigen::VectorXd& QuadrupedRobotRBDL::getJointEfforts() const { return tau_; }
bool QuadrupedRobotRBDL::setJointPosition(const Eigen::VectorXd& q) { q_ = q; return true; }
bool QuadrupedRobotRBDL::setJointVelocity(const Eigen::VectorXd& qd) { qdot_ = qd; return true; }
bool QuadrupedRobotRBDL::setJointEffort(const Eigen::VectorXd& tau) { tau_ = tau; return true; }
bool QuadrupedRobotRBDL::setJointAcceleration(const Eigen::VectorXd& qddot) { qddot_ = qddot; return true; }
bool QuadrupedRobotRBDL::setJointPosition(int i, double q) { if(i<0||i>=q_.size()) return false; q_(i)=q; return true; }
bool QuadrupedRobotRBDL::setJointVelocity(int i, double qd) { if(i<0||i>=qdot_.size()) return false; qdot_(i)=qd; return true; }
bool QuadrupedRobotRBDL::setJointEffort(int i, double tau) { if(i<0||i>=tau_.size()) return false; tau_(i)=tau; return true; }

bool QuadrupedRobotRBDL::getFloatingBasePose(Eigen::Affine3d& pose) const
{
  return getPose(base_name_, pose);
}

bool QuadrupedRobotRBDL::setFloatingBasePose(const Eigen::Affine3d& pose)
{
  Eigen::Vector3d rpy = pose.linear().eulerAngles(0,1,2);
  if(q_.size() < FLOATING_BASE_DOFS) return false;
  q_.segment<3>(0) = pose.translation() - fb_origin_offset_;
  q_.segment<3>(3) = rpy;
  return true;
}

bool QuadrupedRobotRBDL::setFloatingBaseOrientation(const Eigen::Matrix3d& world_R_base)
{
  Eigen::Vector3d rpy = world_R_base.eulerAngles(0,1,2);
  if(q_.size() < FLOATING_BASE_DOFS) return false;
  q_.segment<3>(3) = rpy;
  return true;
}

bool QuadrupedRobotRBDL::setFloatingBaseAngularVelocity(const Eigen::Vector3d& w)
{
  if(qdot_.size() < FLOATING_BASE_DOFS) return false;
  if(floating_base_link_id_ <= 0) {
    qdot_.segment<3>(3) = w;
    return true;
  }
  Eigen::MatrixXd Jfb(6, rbdl_model_.dof_count);
  RigidBodyDynamics::CalcPointJacobian6D(rbdl_model_, q_, floating_base_link_id_, Eigen::Vector3d::Zero(), Jfb, true);
  Eigen::Matrix3d Tphi = Jfb.block<3,3>(0,3);
  qdot_.segment<3>(3) = Tphi.colPivHouseholderQr().solve(w);
  return true;
}

bool QuadrupedRobotRBDL::setFloatingBaseState(const Eigen::Affine3d& pose, const Eigen::Vector6d& twist)
{
  if(!setFloatingBasePose(pose)) return false;
  if(qdot_.size() < FLOATING_BASE_DOFS) return false;
  qdot_.segment<3>(0) = twist.head<3>();
  if(floating_base_link_id_ <= 0) {
    qdot_.segment<3>(3) = twist.tail<3>();
    return true;
  }
  Eigen::MatrixXd Jfb(6, rbdl_model_.dof_count);
  RigidBodyDynamics::CalcPointJacobian6D(rbdl_model_, q_, floating_base_link_id_, Eigen::Vector3d::Zero(), Jfb, true);
  Eigen::Matrix3d Tphi = Jfb.block<3,3>(0,3);
  qdot_.segment<3>(3) = Tphi.colPivHouseholderQr().solve(twist.tail<3>());
  return true;
}

void QuadrupedRobotRBDL::getInertiaMatrix(Eigen::MatrixXd& M) const
{
  M.setZero(getJointNum(), getJointNum());
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(rbdl_model_, q_, M, false);
}

void QuadrupedRobotRBDL::getInertiaInverse(Eigen::MatrixXd& Minv) const
{
  getInertiaMatrix(Minv);
  Minv = Minv.inverse();
}

void QuadrupedRobotRBDL::computeNonlinearTerm(Eigen::VectorXd& h) const
{
  h.setZero(getJointNum());
  RigidBodyDynamics::NonlinearEffects(rbdl_model_, q_, qdot_, h);
}

void QuadrupedRobotRBDL::computeGravityCompensation(Eigen::VectorXd& g) const
{
  g.setZero(getJointNum());
  RigidBodyDynamics::NonlinearEffects(rbdl_model_, q_, Eigen::VectorXd::Zero(qdot_.size()), g);
}

void QuadrupedRobotRBDL::computeInverseDynamics(Eigen::VectorXd& tau) const
{
  tau.setZero(getJointNum());
  RigidBodyDynamics::InverseDynamics(rbdl_model_, q_, qdot_, qddot_, tau);
}

bool QuadrupedRobotRBDL::isFloatingBase() const
{
  return getJointNum() >= FLOATING_BASE_DOFS;
}

bool QuadrupedRobotRBDL::getOrientation(const std::string& frame, Eigen::Matrix3d& R) const
{
  Eigen::Affine3d T;
  if(!getPose(frame, T)) return false;
  R = T.linear();
  return true;
}

bool QuadrupedRobotRBDL::getOrientation(const std::string& source_frame, const std::string& target_frame, Eigen::Matrix3d& R) const
{
  Eigen::Affine3d T;
  if(!getPose(source_frame, target_frame, T)) return false;
  R = T.linear();
  return true;
}

const urdf::ModelInterface& QuadrupedRobotRBDL::getUrdf() const { return urdf_model_; }
const std::string& QuadrupedRobotRBDL::getUrdfString() const { return urdf_string_; }
const std::string& QuadrupedRobotRBDL::getSrdfString() const { return srdf_string_; }

bool QuadrupedRobotRBDL::getJointLimits(Eigen::VectorXd& qmin, Eigen::VectorXd& qmax) const
{
  qmin.setConstant(getJointNum(), -std::numeric_limits<double>::infinity());
  qmax.setConstant(getJointNum(),  std::numeric_limits<double>::infinity());
  for(const auto& j : urdf_model_.joints_) {
    if(j.second->type == urdf::Joint::FIXED) continue;
    auto it = joint_idx_.find(j.first);
    if(it == joint_idx_.end()) continue;
    if(j.second->limits) {
      qmin(it->second) = j.second->limits->lower;
      qmax(it->second) = j.second->limits->upper;
    }
  }
  return true;
}

bool QuadrupedRobotRBDL::getVelocityLimits(Eigen::VectorXd& qdot_max) const
{
  qdot_max.setConstant(getJointNum(), std::numeric_limits<double>::infinity());
  for(const auto& j : urdf_model_.joints_) {
    if(j.second->type == urdf::Joint::FIXED) continue;
    auto it = joint_idx_.find(j.first);
    if(it == joint_idx_.end()) continue;
    if(j.second->limits) {
      qdot_max(it->second) = j.second->limits->velocity;
    }
  }
  return true;
}

bool QuadrupedRobotRBDL::getEffortLimits(Eigen::VectorXd& tau_max) const
{
  tau_max.setConstant(getJointNum(), std::numeric_limits<double>::infinity());
  for(const auto& j : urdf_model_.joints_) {
    if(j.second->type == urdf::Joint::FIXED) continue;
    auto it = joint_idx_.find(j.first);
    if(it == joint_idx_.end()) continue;
    if(j.second->limits) {
      tau_max(it->second) = j.second->limits->effort;
    }
  }
  return true;
}

} // namespace wolf_wbid
