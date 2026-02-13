/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#include <wolf_wbid/ros2/postural.h>
#include <wolf_controller_utils/ros2_param_getter.h>

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
  if(rt_pub_)
  {
    rt_pub_->msg_.name.resize(n);
    rt_pub_->msg_.position_actual.resize(n);
    rt_pub_->msg_.position_reference.resize(n);
    rt_pub_->msg_.velocity_actual.resize(n);
    rt_pub_->msg_.velocity_reference.resize(n);
    rt_pub_->msg_.position_error.resize(n);
    rt_pub_->msg_.velocity_error.resize(n);
  }

  buffer_lambda1_ = getLambda1();
  buffer_lambda2_ = getLambda2();
  buffer_weight_diag_ = getWeightDiag();
}

void PosturalImpl::registerReconfigurableVariables()
{
  TaskWrapperInterface::setLambda1(getLambda1());
  TaskWrapperInterface::setLambda2(getLambda2());
  TaskWrapperInterface::setWeightDiag(getWeightDiag());
}

void PosturalImpl::loadParams()
{
  double lambda1 = getLambda1();
  double lambda2 = getLambda2();
  double weight = getWeightDiag();

  lambda1 = wolf_controller_utils::get_double_parameter_from_remote_node(
      "wolf_controller/gains." + task_name_ + ".lambda1", lambda1);
  lambda2 = wolf_controller_utils::get_double_parameter_from_remote_node(
      "wolf_controller/gains." + task_name_ + ".lambda2", lambda2);
  weight = wolf_controller_utils::get_double_parameter_from_remote_node(
      "wolf_controller/gains." + task_name_ + ".weight", weight);

  if(lambda1 < 0.0 || lambda2 < 0.0 || weight < 0.0)
    throw std::runtime_error("PosturalImpl::loadParams(): lambda/weight must be >= 0");

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
}

void PosturalImpl::updateCost(const Eigen::VectorXd& x)
{
  cost_ = computeCost(x);
}

void PosturalImpl::publish()
{
  if(!rt_pub_) return;

  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = "Joints";
    rt_pub_->msg_.header.stamp = task_nh_->now();

    const auto& names = jointNames();
    const auto& q = actualQ();
    const auto& qdot = actualQdot();
    const auto& qref = refQ();
    const auto& qdref = refQdotCached();
    const auto& e = posError();
    const auto& edot = velError();

    const std::size_t n = static_cast<std::size_t>(q.size());

    rt_pub_->msg_.name.resize(n);
    rt_pub_->msg_.position_actual.resize(n);
    rt_pub_->msg_.position_reference.resize(n);
    rt_pub_->msg_.velocity_actual.resize(n);
    rt_pub_->msg_.velocity_reference.resize(n);
    rt_pub_->msg_.position_error.resize(n);
    rt_pub_->msg_.velocity_error.resize(n);

    for(std::size_t i = 0; i < n; ++i)
    {
      rt_pub_->msg_.name[i] = (i < names.size()) ? names[i] : std::string("");
      rt_pub_->msg_.position_actual[i] = q(i);
      rt_pub_->msg_.position_reference[i] = qref(i);
      rt_pub_->msg_.velocity_actual[i] = qdot(i);
      rt_pub_->msg_.velocity_reference[i] = (qdref.size() == qdot.size()) ? qdref(i) : 0.0;
      rt_pub_->msg_.position_error[i] = e(i);
      rt_pub_->msg_.velocity_error[i] = edot(i);
    }

    rt_pub_->msg_.cost = cost_;
    rt_pub_->unlockAndPublish();
  }
}

bool PosturalImpl::reset()
{
  const bool ok = PosturalTask::reset();

  buffer_lambda1_ = getLambda1();
  buffer_lambda2_ = getLambda2();
  buffer_weight_diag_ = getWeightDiag();

  return ok;
}

} // namespace wolf_wbid
