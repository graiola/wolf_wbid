/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#include <wolf_wbid/ros2/momentum.h>
#include <wolf_controller_utils/converters.h>
#include <wolf_controller_utils/ros2_param_getter.h>

#include <cmath>
#include <limits>
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
  const Eigen::Matrix3d K = getMomentumGain();

  const auto declare_or_get = [this](const std::string& name, double default_value) {
    if(!task_nh_->has_parameter(name))
      task_nh_->declare_parameter<double>(name, default_value);
    double value = default_value;
    task_nh_->get_parameter(name, value);
    return value;
  };

  TaskWrapperInterface::setLambda1(declare_or_get("set_lambda_1", getLambda()));
  TaskWrapperInterface::setWeightDiag(declare_or_get("set_weight_diag", getWeight()(0,0)));
  TaskWrapperInterface::setKpRoll(declare_or_get("K_roll", K(0,0)));
  TaskWrapperInterface::setKpPitch(declare_or_get("K_pitch", K(1,1)));
  TaskWrapperInterface::setKpYaw(declare_or_get("K_yaw", K(2,2)));

  param_cb_handle_ = task_nh_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "ok";

        auto reject = [&](const std::string& reason) {
          result.successful = false;
          result.reason = reason;
        };

        for(const auto& param : params)
        {
          if(param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
            continue;

          const std::string& name = param.get_name();
          const double value = param.as_double();
          if(!std::isfinite(value) || value < 0.0)
          {
            reject("Task parameters must be finite and >= 0");
            break;
          }

          if(name == "set_lambda_1") TaskWrapperInterface::setLambda1(value);
          else if(name == "set_weight_diag") TaskWrapperInterface::setWeightDiag(value);
          else if(name == "K_roll") TaskWrapperInterface::setKpRoll(value);
          else if(name == "K_pitch") TaskWrapperInterface::setKpPitch(value);
          else if(name == "K_yaw") TaskWrapperInterface::setKpYaw(value);
        }
        return result;
      });
}

void AngularMomentumImpl::loadParams()
{
  double lambda1 = getLambda();
  double weight  = getWeight()(0,0);

  const auto get_task_param = [this](const std::string& key, double default_value) {
    return wolf_controller_utils::get_double_parameter_from_remote_controller_node(
        robot_name_, "gains." + task_name_ + "." + key, default_value);
  };

  lambda1 = get_task_param("lambda1", lambda1);
  weight  = get_task_param("weight", weight);

  if(!std::isfinite(lambda1) || lambda1 < 0.0) lambda1 = getLambda();
  if(!std::isfinite(weight)  || weight  < 0.0) weight  = getWeight()(0,0);

  buffer_lambda1_ = lambda1;
  buffer_weight_diag_ = weight;

  setLambda(lambda1);
  Eigen::VectorXd w = Eigen::VectorXd::Constant(3, weight);
  setWeight(w);

  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  bool use_identity = false;
  for(unsigned int i = 0; i < wolf_controller_utils::_rpy.size(); i++)
  {
    K(i, i) = get_task_param("K." + wolf_controller_utils::_rpy[i], std::numeric_limits<double>::quiet_NaN());
    if(!std::isfinite(K(i, i)) || K(i, i) < 0.0)
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
