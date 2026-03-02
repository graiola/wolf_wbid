/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#include <wolf_wbid/ros2/com.h>
#include <wolf_controller_utils/ros2_param_getter.h>

#include <cmath>
#include <limits>
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
  const Eigen::Matrix3d Kp = getKp();
  const Eigen::Matrix3d Kd = getKd();

  const auto declare_or_get = [this](const std::string& name, double default_value) {
    if(!task_nh_->has_parameter(name))
      task_nh_->declare_parameter<double>(name, default_value);
    double value = default_value;
    task_nh_->get_parameter(name, value);
    return value;
  };

  TaskWrapperInterface::setLambda1(declare_or_get("set_lambda_1", getLambda()));
  TaskWrapperInterface::setLambda2(declare_or_get("set_lambda_2", getLambda2()));
  TaskWrapperInterface::setWeightDiag(declare_or_get("set_weight_diag", getWeight()(0,0)));

  TaskWrapperInterface::setKpX(declare_or_get("kp_x", Kp(0,0)));
  TaskWrapperInterface::setKpY(declare_or_get("kp_y", Kp(1,1)));
  TaskWrapperInterface::setKpZ(declare_or_get("kp_z", Kp(2,2)));

  TaskWrapperInterface::setKdX(declare_or_get("kd_x", Kd(0,0)));
  TaskWrapperInterface::setKdY(declare_or_get("kd_y", Kd(1,1)));
  TaskWrapperInterface::setKdZ(declare_or_get("kd_z", Kd(2,2)));

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
          else if(name == "set_lambda_2") TaskWrapperInterface::setLambda2(value);
          else if(name == "set_weight_diag") TaskWrapperInterface::setWeightDiag(value);
          else if(name == "kp_x") TaskWrapperInterface::setKpX(value);
          else if(name == "kp_y") TaskWrapperInterface::setKpY(value);
          else if(name == "kp_z") TaskWrapperInterface::setKpZ(value);
          else if(name == "kd_x") TaskWrapperInterface::setKdX(value);
          else if(name == "kd_y") TaskWrapperInterface::setKdY(value);
          else if(name == "kd_z") TaskWrapperInterface::setKdZ(value);
        }
        return result;
      });
}

void ComImpl::loadParams()
{
  double lambda1 = getLambda();
  double lambda2 = getLambda2();
  double weight  = getWeight()(0,0);

  const auto get_com_param = [&](const std::string& key, double default_value) {
    const double value_lower = wolf_controller_utils::get_double_parameter_from_remote_controller_node(
        robot_name_, "gains.com." + key, std::numeric_limits<double>::quiet_NaN());
    if(std::isfinite(value_lower))
      return value_lower;

    // Backward-compatibility with legacy "CoM" naming.
    return wolf_controller_utils::get_double_parameter_from_remote_controller_node(
        robot_name_, "gains.CoM." + key, default_value);
  };

  lambda1 = get_com_param("lambda1", lambda1);
  lambda2 = get_com_param("lambda2", lambda2);
  weight  = get_com_param("weight",  weight);

  if(!std::isfinite(lambda1) || lambda1 < 0.0) lambda1 = getLambda();
  if(!std::isfinite(lambda2) || lambda2 < 0.0) lambda2 = getLambda2();
  if(!std::isfinite(weight)  || weight  < 0.0) weight  = getWeight()(0,0);

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
    Kp(i, i) = get_com_param("Kp." + wolf_controller_utils::_xyz[i], std::numeric_limits<double>::quiet_NaN());
    Kd(i, i) = get_com_param("Kd." + wolf_controller_utils::_xyz[i], std::numeric_limits<double>::quiet_NaN());

    if(!std::isfinite(Kp(i, i)) || !std::isfinite(Kd(i, i)) || Kp(i, i) < 0.0 || Kd(i, i) < 0.0)
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
