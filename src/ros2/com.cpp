/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#include <wolf_wbid/ros2/com.h>
#include <wolf_controller_utils/ros2_param_getter.h>

#include <stdexcept>

namespace wolf_wbid {

ComImpl::ComImpl(const std::string& robot_name,
                 const std::string& task_id,
                 QuadrupedRobot& robot,
                 const IDVariables& vars,
                 const double& period)
  : Com(robot_name, task_id, robot, vars, period)
  , TaskRosHandler<wolf_msgs::msg::ComTask>(task_id, robot_name, period)
{
  tmp_vector3d_.setZero();
  buffer_reference_pos_.initRT(tmp_vector3d_);
  buffer_reference_vel_.initRT(tmp_vector3d_);

  reference_sub_ = task_nh_->create_subscription<wolf_msgs::msg::Com>(
      "reference/" + task_id,
      1000,
      std::bind(&ComImpl::referenceCallback, this, std::placeholders::_1));
}

void ComImpl::registerReconfigurableVariables()
{
  const double lambda1 = getLambda();
  const double lambda2 = getLambda2();
  const double weight  = getWeight()(0,0);
  const Eigen::Matrix3d Kp = getKp();
  const Eigen::Matrix3d Kd = getKd();

  TaskWrapperInterface::setLambda1(lambda1);
  TaskWrapperInterface::setLambda2(lambda2);
  TaskWrapperInterface::setWeightDiag(weight);

  TaskWrapperInterface::setKpX(Kp(0,0));
  TaskWrapperInterface::setKpY(Kp(1,1));
  TaskWrapperInterface::setKpZ(Kp(2,2));

  TaskWrapperInterface::setKdX(Kd(0,0));
  TaskWrapperInterface::setKdY(Kd(1,1));
  TaskWrapperInterface::setKdZ(Kd(2,2));
}

void ComImpl::loadParams()
{
  double lambda1 = getLambda();
  double lambda2 = getLambda2();
  double weight  = getWeight()(0,0);

  lambda1 = wolf_controller_utils::get_double_parameter_from_remote_node(
      "wolf_controller/gains." + task_name_ + ".lambda1", lambda1);
  lambda2 = wolf_controller_utils::get_double_parameter_from_remote_node(
      "wolf_controller/gains." + task_name_ + ".lambda2", lambda2);
  weight = wolf_controller_utils::get_double_parameter_from_remote_node(
      "wolf_controller/gains." + task_name_ + ".weight", weight);

  if(lambda1 < 0.0 || lambda2 < 0.0 || weight < 0.0)
    throw std::runtime_error("ComImpl::loadParams(): lambda/weight must be >= 0");

  buffer_lambda1_ = lambda1;
  buffer_lambda2_ = lambda2;
  buffer_weight_diag_ = weight;

  setLambda(lambda1, lambda2);
  Eigen::VectorXd w = Eigen::VectorXd::Constant(3, weight);
  setWeight(w);

  Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Kd = Eigen::Matrix3d::Zero();
  bool use_identity = false;

  for(unsigned int i = 0; i < wolf_controller_utils::_xyz.size(); i++)
  {
    Kp(i, i) = wolf_controller_utils::get_double_parameter_from_remote_node(
        "wolf_controller/gains." + task_name_ + ".Kp." + wolf_controller_utils::_xyz[i], 0.0);
    Kd(i, i) = wolf_controller_utils::get_double_parameter_from_remote_node(
        "wolf_controller/gains." + task_name_ + ".Kd." + wolf_controller_utils::_xyz[i], 0.0);

    if(Kp(i, i) < 0.0 || Kd(i, i) < 0.0)
      use_identity = true;
  }

  if(use_identity)
  {
    Kp = Eigen::Matrix3d::Identity();
    Kd = Eigen::Matrix3d::Identity();
  }

  buffer_kp_x_ = Kp(0,0);
  buffer_kp_y_ = Kp(1,1);
  buffer_kp_z_ = Kp(2,2);

  buffer_kd_x_ = Kd(0,0);
  buffer_kd_y_ = Kd(1,1);
  buffer_kd_z_ = Kd(2,2);

  setKp(Kp);
  setKd(Kd);
}

void ComImpl::applyExternalKnobs()
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_.load(), buffer_lambda2_.load());

  if(OPTIONS.set_ext_weight)
  {
    Eigen::VectorXd w = Eigen::VectorXd::Constant(3, buffer_weight_diag_.load());
    setWeight(w);
  }

  if(OPTIONS.set_ext_gains)
  {
    Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Kd = Eigen::Matrix3d::Zero();

    Kp(0,0) = buffer_kp_x_.load();
    Kp(1,1) = buffer_kp_y_.load();
    Kp(2,2) = buffer_kp_z_.load();

    Kd(0,0) = buffer_kd_x_.load();
    Kd(1,1) = buffer_kd_y_.load();
    Kd(2,2) = buffer_kd_z_.load();

    setKp(Kp);
    setKd(Kd);
  }
}

void ComImpl::applyExternalReference()
{
  if(!OPTIONS.set_ext_reference)
    return;

  setReference(*buffer_reference_pos_.readFromRT(),
               *buffer_reference_vel_.readFromRT());
}

void ComImpl::updateCost(const Eigen::VectorXd& x)
{
  const Eigen::VectorXd r = A() * x - b();
  const Eigen::VectorXd wd = wDiag();
  cost_ = 0.5 * (r.array().square() * wd.array()).sum();
}

void ComImpl::publish()
{
  if(!rt_pub_) return;

  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = getBaseLink();
    rt_pub_->msg_.header.stamp = task_nh_->now();

    getActualPose(tmp_vector3d_);
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.position_actual);

    getActualVelocity(tmp_vector3d_);
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.velocity_actual);

    getReference(tmp_vector3d_);
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.position_reference);

    tmp_vector3d_ = getCachedVelocityReference();
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.velocity_reference);

    rt_pub_->msg_.cost = cost_;
    rt_pub_->unlockAndPublish();
  }
}

bool ComImpl::reset()
{
  const bool ok = ComTask::reset();

  getActualPose(tmp_vector3d_);
  buffer_reference_pos_.writeFromNonRT(tmp_vector3d_);

  tmp_vector3d_.setZero();
  buffer_reference_vel_.writeFromNonRT(tmp_vector3d_);

  return ok;
}

void ComImpl::referenceCallback(const wolf_msgs::msg::Com::SharedPtr msg)
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();

  pos.x() = msg->position.x;
  pos.y() = msg->position.y;
  pos.z() = msg->position.z;

  vel.x() = msg->velocity.x;
  vel.y() = msg->velocity.y;
  vel.z() = msg->velocity.z;

  buffer_reference_pos_.writeFromNonRT(pos);
  buffer_reference_vel_.writeFromNonRT(vel);

  last_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
}

} // namespace wolf_wbid
