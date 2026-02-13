/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#include <wolf_wbid/ros2/momentum.h>
#include <wolf_controller_utils/converters.h>
#include <wolf_controller_utils/ros2_param_getter.h>

#include <stdexcept>

namespace wolf_wbid {

AngularMomentumImpl::AngularMomentumImpl(const std::string& robot_name,
                                         const std::string& task_id,
                                         QuadrupedRobot& robot,
                                         const IDVariables& vars,
                                         const double& period)
  : AngularMomentum(robot_name, task_id, robot, vars, period)
  , TaskRosHandler<wolf_msgs::msg::CartesianTask>(task_id, robot_name, period)
{
}

void AngularMomentumImpl::registerReconfigurableVariables()
{
  const double lambda1 = getLambda();
  const double weight  = getWeight()(0,0);
  const Eigen::Matrix3d K = getMomentumGain();

  TaskWrapperInterface::setLambda1(lambda1);
  TaskWrapperInterface::setWeightDiag(weight);
  TaskWrapperInterface::setKpRoll(K(0,0));
  TaskWrapperInterface::setKpPitch(K(1,1));
  TaskWrapperInterface::setKpYaw(K(2,2));
}

void AngularMomentumImpl::loadParams()
{
  double lambda1 = getLambda();
  double weight  = getWeight()(0,0);

  lambda1 = wolf_controller_utils::get_double_parameter_from_remote_node(
      "wolf_controller/gains." + task_name_ + ".lambda1", lambda1);
  weight = wolf_controller_utils::get_double_parameter_from_remote_node(
      "wolf_controller/gains." + task_name_ + ".weight", weight);

  if(lambda1 < 0.0 || weight < 0.0)
    throw std::runtime_error("AngularMomentumImpl::loadParams(): lambda/weight must be >= 0");

  buffer_lambda1_ = lambda1;
  buffer_weight_diag_ = weight;

  setLambda(lambda1);
  Eigen::VectorXd w = Eigen::VectorXd::Constant(3, weight);
  setWeight(w);

  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  bool use_identity = false;
  for(unsigned int i = 0; i < wolf_controller_utils::_rpy.size(); i++)
  {
    K(i, i) = wolf_controller_utils::get_double_parameter_from_remote_node(
        "wolf_controller/gains." + task_name_ + ".K." + wolf_controller_utils::_rpy[i], K(i, i));
    if(K(i, i) < 0.0)
      use_identity = true;
  }

  if(use_identity)
    K = Eigen::Matrix3d::Identity();

  buffer_kp_roll_ = K(0,0);
  buffer_kp_pitch_ = K(1,1);
  buffer_kp_yaw_ = K(2,2);

  setMomentumGain(K);
}

void AngularMomentumImpl::applyExternalKnobs()
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_.load());

  if(OPTIONS.set_ext_weight)
  {
    Eigen::VectorXd w = Eigen::VectorXd::Constant(3, buffer_weight_diag_.load());
    setWeight(w);
  }

  if(OPTIONS.set_ext_gains)
  {
    Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    K(0,0) = buffer_kp_roll_.load();
    K(1,1) = buffer_kp_pitch_.load();
    K(2,2) = buffer_kp_yaw_.load();
    setMomentumGain(K);
  }
}

void AngularMomentumImpl::applyExternalReference()
{
}

void AngularMomentumImpl::updateCost(const Eigen::VectorXd& x)
{
  const Eigen::VectorXd r = A() * x - b();
  const Eigen::VectorXd wd = wDiag();
  cost_ = 0.5 * (r.array().square() * wd.array()).sum();
}

void AngularMomentumImpl::publish()
{
  if(!rt_pub_) return;

  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = WORLD_FRAME_NAME;
    rt_pub_->msg_.header.stamp = task_nh_->now();

    const Eigen::Vector3d L_act = getActualAngularMomentum();
    const Eigen::Vector3d L_ref = getCachedDesiredAngularMomentum();

    wolf_controller_utils::vector3dToVector3(L_act, rt_pub_->msg_.rpy_actual);
    wolf_controller_utils::vector3dToVector3(L_ref, rt_pub_->msg_.rpy_reference);

    rt_pub_->msg_.cost = cost_;
    rt_pub_->unlockAndPublish();
  }
}

bool AngularMomentumImpl::reset()
{
  return AngularMomentumTask::reset();
}

} // namespace wolf_wbid
