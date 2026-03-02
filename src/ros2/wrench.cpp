/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#include <wolf_wbid/ros2/wrench.h>
#include <wolf_controller_utils/ros2_param_getter.h>

#include <cmath>
#include <stdexcept>

namespace wolf_wbid {

WrenchImpl::WrenchImpl(const std::string& robot_name,
                       const std::string& task_id,
                       const std::string& contact_name,
                       const IDVariables& vars,
                       const double& period)
  : Wrench(robot_name, task_id, contact_name, vars, period, 1.0)
  , TaskRosHandler<wolf_msgs::msg::WrenchTask>(task_id, robot_name, period)
  , vars_(vars)
{
  buffer_reference_force_.initRT(Eigen::Vector3d::Zero());

  reference_sub_ = task_nh_->create_subscription<wolf_msgs::msg::Wrench>(
      "reference/" + task_id,
      1000,
      std::bind(&WrenchImpl::referenceCallback, this, std::placeholders::_1));

  buffer_lambda1_ = this->getLambda1();
  buffer_weight_diag_ = this->weight();
}

void WrenchImpl::registerReconfigurableVariables()
{
  const auto declare_or_get = [this](const std::string& name, double default_value) {
    if(!task_nh_->has_parameter(name))
      task_nh_->declare_parameter<double>(name, default_value);
    double value = default_value;
    task_nh_->get_parameter(name, value);
    return value;
  };

  TaskWrapperInterface::setLambda1(declare_or_get("set_lambda_1", this->getLambda1()));
  TaskWrapperInterface::setWeightDiag(declare_or_get("set_weight_diag", this->weight()));

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
        }
        return result;
      });
}

void WrenchImpl::loadParams()
{
  double lambda1 = this->getLambda1();
  double weight = this->weight();

  const auto get_task_param = [this](const std::string& key, double default_value) {
    return wolf_controller_utils::get_double_parameter_from_remote_controller_node(
        robot_name_, "gains." + task_name_ + "." + key, default_value);
  };

  lambda1 = get_task_param("lambda1", lambda1);
  weight  = get_task_param("weight", weight);

  if(!std::isfinite(lambda1) || lambda1 < 0.0) lambda1 = this->getLambda1();
  if(!std::isfinite(weight)  || weight  < 0.0) weight  = this->weight();

  buffer_lambda1_ = lambda1;
  buffer_weight_diag_ = weight;

  this->setLambda(lambda1, 0.0);
  this->setWeight(weight);
}

void WrenchImpl::applyExternalKnobs()
{
  if(OPTIONS.set_ext_lambda)
    this->setLambda(buffer_lambda1_.load(), 0.0);

  if(OPTIONS.set_ext_weight)
    this->setWeight(buffer_weight_diag_.load());
}

void WrenchImpl::applyExternalReference()
{
  if(OPTIONS.set_ext_reference)
    this->setReference(*buffer_reference_force_.readFromRT());
}

void WrenchImpl::updateCost(const Eigen::VectorXd& x)
{
  const int off = vars_.contactOffset(contactName());
  if(off < 0 || off + 3 > x.size())
    throw std::runtime_error("WrenchImpl::updateCost(): contact block out of range for " + contactName());

  const Eigen::Vector3d f_act = x.segment<3>(off);
  const Eigen::Vector3d f_ref = this->reference();
  const Eigen::Vector3d e = f_act - f_ref;

  last_f_act_ = f_act;
  has_last_f_act_ = true;

  cost_ = 0.5 * this->weight() * e.squaredNorm();
}

void WrenchImpl::publish()
{
  if(!rt_pub_) return;

  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = WORLD_FRAME_NAME;
    rt_pub_->msg_.header.stamp = task_nh_->now();

    const Eigen::Vector3d f_ref = this->reference();
    rt_pub_->msg_.wrench_reference.force.x = f_ref.x();
    rt_pub_->msg_.wrench_reference.force.y = f_ref.y();
    rt_pub_->msg_.wrench_reference.force.z = f_ref.z();
    rt_pub_->msg_.wrench_reference.torque.x = 0.0;
    rt_pub_->msg_.wrench_reference.torque.y = 0.0;
    rt_pub_->msg_.wrench_reference.torque.z = 0.0;

    if(has_last_f_act_)
    {
      rt_pub_->msg_.wrench_actual.force.x = last_f_act_.x();
      rt_pub_->msg_.wrench_actual.force.y = last_f_act_.y();
      rt_pub_->msg_.wrench_actual.force.z = last_f_act_.z();
    }
    else
    {
      rt_pub_->msg_.wrench_actual.force.x = 0.0;
      rt_pub_->msg_.wrench_actual.force.y = 0.0;
      rt_pub_->msg_.wrench_actual.force.z = 0.0;
    }

    rt_pub_->msg_.wrench_actual.torque.x = 0.0;
    rt_pub_->msg_.wrench_actual.torque.y = 0.0;
    rt_pub_->msg_.wrench_actual.torque.z = 0.0;
    rt_pub_->msg_.cost = cost_;

    rt_pub_->unlockAndPublish();
  }
}

bool WrenchImpl::reset()
{
  this->WrenchTask::reset();
  buffer_reference_force_.initRT(Eigen::Vector3d::Zero());
  has_last_f_act_ = false;
  last_f_act_.setZero();

  buffer_lambda1_ = this->getLambda1();
  buffer_weight_diag_ = this->weight();
  return true;
}

void WrenchImpl::referenceCallback(const wolf_msgs::msg::Wrench::SharedPtr msg)
{
  Eigen::Vector3d f = Eigen::Vector3d::Zero();
  f.x() = msg->wrench.force.x;
  f.y() = msg->wrench.force.y;
  f.z() = msg->wrench.force.z;

  buffer_reference_force_.writeFromNonRT(f);
  last_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
}

} // namespace wolf_wbid
