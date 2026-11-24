/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#include <wolf_wbid/ros2/postural.h>
#include <wolf_wbid/core/quadruped_robot.h>
#include <wolf_controller_utils/ros2_param_getter.h>

#include <cmath>
#include <stdexcept>

namespace wolf_wbid {

PosturalImpl::PosturalImpl(const std::string& robot_name,
                           const std::string& task_id,
                           QuadrupedRobot& robot,
                           const IDVariables& vars,
                           const double& period)
  : Postural(robot_name, task_id, robot, vars, period)
  , TaskRosHandler<wolf_msgs::msg::PosturalTask>(task_id, robot_name, period)
{
  const std::size_t n = static_cast<std::size_t>(taskSize());
  reference_positions_.setZero(n);
  reference_velocities_.setZero(n);
  for (const auto& name : jointNames()) {
    const auto index = robot.getJointIndex(name);
    if (index < FLOATING_BASE_DOFS || index >= taskSize())
      throw std::invalid_argument("Postural reference joint is absent from the model: " + name);
    reference_joint_indices_.push_back(static_cast<std::size_t>(index));
  }
  for (const auto& leg : robot.getLegNames()) {
    for (const auto index : robot.getLimbJointsIds(leg)) {
      const auto found = std::find(reference_joint_indices_.begin(), reference_joint_indices_.end(), index);
      if (found == reference_joint_indices_.end())
        throw std::invalid_argument("Postural leg joint is absent from the task model");
      leg_reference_indices_.push_back(static_cast<std::size_t>(found - reference_joint_indices_.begin()));
    }
  }
  reference_buffer_.initRT(NamedJointReference{});
  reference_sub_ = task_nh_->create_subscription<wolf_msgs::msg::Postural>(
      "reference/" + task_id, 1,
      std::bind(&PosturalImpl::referenceCallback, this, std::placeholders::_1));
  buffer_lambda1_ = getLambda1();
  buffer_lambda2_ = getLambda2();
  buffer_weight_diag_ = getWeightDiag();
}

void PosturalImpl::registerReconfigurableVariables()
{
  const auto declare_or_get = [this](const std::string& name, double default_value) {
    if(!task_nh_->has_parameter(name))
      task_nh_->declare_parameter<double>(name, default_value);
    double value = default_value;
    task_nh_->get_parameter(name, value);
    return value;
  };

  TaskWrapperInterface::setLambda1(declare_or_get("set_lambda_1", getLambda1()));
  TaskWrapperInterface::setLambda2(declare_or_get("set_lambda_2", getLambda2()));
  TaskWrapperInterface::setWeightDiag(declare_or_get("set_weight_diag", getWeightDiag()));

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
        }
        return result;
      });
}

void PosturalImpl::loadParams()
{
  double lambda1 = getLambda1();
  double lambda2 = getLambda2();
  double weight = getWeightDiag();

  const auto get_task_param = [this](const std::string& key, double default_value) {
    return wolf_controller_utils::get_double_parameter_from_remote_controller_node(
        robot_name_, "gains." + task_name_ + "." + key, default_value);
  };

  lambda1 = get_task_param("lambda1", lambda1);
  lambda2 = get_task_param("lambda2", lambda2);
  weight  = get_task_param("weight", weight);

  if(!std::isfinite(lambda1) || lambda1 < 0.0) lambda1 = getLambda1();
  if(!std::isfinite(lambda2) || lambda2 < 0.0) lambda2 = getLambda2();
  if(!std::isfinite(weight)  || weight  < 0.0) weight  = getWeightDiag();

  buffer_lambda1_ = lambda1;
  buffer_lambda2_ = lambda2;
  buffer_weight_diag_ = weight;

  setLambda(lambda1, lambda2);
  PosturalTask::setWeightDiag(weight);
}

void PosturalImpl::applyExternalKnobs()
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_.load(), buffer_lambda2_.load());

  if(OPTIONS.set_ext_weight)
    PosturalTask::setWeightDiag(buffer_weight_diag_.load());
}

void PosturalImpl::applyExternalReference()
{
  if (!OPTIONS.set_ext_reference)
    return;
  const auto* reference = reference_buffer_.readFromRT();
  if (!reference->valid || reference->generation != externalReferenceGeneration())
    return;

  // The core task includes floating-base rows; received names identify actuated
  // joints only. Preserve unnamed arm joints and the current base reference.
  reference_positions_ = refQ();
  reference_velocities_ = refQdotCached();
  for (std::size_t i = 0; i < reference->indices.size(); ++i) {
    const auto index = reference_joint_indices_[reference->indices[i]];
    reference_positions_(index) = reference->positions[i];
    reference_velocities_(index) = reference->velocities[i];
  }
  PosturalTask::setReference(reference_positions_, reference_velocities_);
}

void PosturalImpl::referenceCallback(const wolf_msgs::msg::Postural::SharedPtr msg)
{
  const auto generation = externalReferenceGeneration();
  NamedJointReference reference;
  if (!resolveNamedJointReference(jointNames(), msg->name, msg->positions,
                                 msg->velocities, reference)) {
    RCLCPP_WARN_THROTTLE(task_nh_->get_logger(), *task_nh_->get_clock(), 2000,
                        "Rejected invalid named postural reference");
    return;
  }
  reference.generation = generation;
  const bool complete_legs = coversRequiredJoints(reference, leg_reference_indices_);
  if (OPTIONS.mpc_reference_mode && !complete_legs) {
    RCLCPP_WARN_THROTTLE(task_nh_->get_logger(), *task_nh_->get_clock(), 2000,
                        "MPC postural references must include every leg joint");
    return;
  }
  if (!complete_legs) {
    invalidateExternalReference();
    reference.generation = externalReferenceGeneration();
  }
  reference_buffer_.writeFromNonRT(reference);
  if (complete_legs)
    markExternalReferenceReceived(reference.generation);
}

void PosturalImpl::updateCost(const Eigen::VectorXd& x)
{
  cost_ = computeCost(x);
}

void PosturalImpl::publish()
{
  if(!pub_) return;

  wolf_msgs::msg::PosturalTask msg;
  msg.header.frame_id = "Joints";
  msg.header.stamp = task_nh_->now();

  const auto& names = jointNames();
  const auto& q = actualQ();
  const auto& qdot = actualQdot();
  const auto& qref = refQ();
  const auto& qdref = refQdotCached();
  const auto& e = posError();
  const auto& edot = velError();

  const std::size_t n = static_cast<std::size_t>(q.size());

  msg.name.resize(n);
  msg.position_actual.resize(n);
  msg.position_reference.resize(n);
  msg.velocity_actual.resize(n);
  msg.velocity_reference.resize(n);
  msg.position_error.resize(n);
  msg.velocity_error.resize(n);

  for(std::size_t i = 0; i < n; ++i)
  {
    msg.name[i] = (i < names.size()) ? names[i] : std::string("");
    msg.position_actual[i] = q(i);
    msg.position_reference[i] = qref(i);
    msg.velocity_actual[i] = qdot(i);
    msg.velocity_reference[i] = (qdref.size() == qdot.size()) ? qdref(i) : 0.0;
    msg.position_error[i] = e(i);
    msg.velocity_error[i] = edot(i);
  }

  msg.cost = cost_;
  pub_->publish(std::move(msg));
}

bool PosturalImpl::reset()
{
  // Invalidate in-flight callbacks without writing the non-RT buffer from the
  // control thread. The next valid message starts a new reference generation.
  if (!OPTIONS.mpc_reference_mode) {
    invalidateExternalReference();
  }
  const bool ok = PosturalTask::reset();

  buffer_lambda1_ = getLambda1();
  buffer_lambda2_ = getLambda2();
  buffer_weight_diag_ = getWeightDiag();

  return ok;
}

} // namespace wolf_wbid
