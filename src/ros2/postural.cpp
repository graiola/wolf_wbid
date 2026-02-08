/**
 * @file postural.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the postural task wrapper for ROS
 */

// WoLF
#include <wolf_wbid/ros2/postural.h>
#include <wolf_controller_utils/ros2_param_getter.h>

using namespace wolf_controller_utils;
using namespace wolf_wbid;

PosturalImpl::PosturalImpl(const std::string& robot_name, QuadrupedRobot& robot,
                           OpenSoT::AffineHelper qddot, const std::string& task_id, const double& period)
  :Postural(robot_name,robot,qddot,task_id,period)
  ,TaskRosHandler<wolf_msgs::msg::PosturalTask>(task_id,robot_name,period)
{
  const unsigned int& size = getActualPositions().size();
  tmp_vectorXd_.resize(size);
  rt_pub_->msg_.name.resize(size);
  rt_pub_->msg_.position_actual.resize(size);
  rt_pub_->msg_.velocity_actual.resize(size);
  rt_pub_->msg_.position_reference.resize(size);
  rt_pub_->msg_.velocity_reference.resize(size);
  rt_pub_->msg_.position_error.resize(size);
  rt_pub_->msg_.velocity_error.resize(size);
}

void PosturalImpl::registerReconfigurableVariables()
{
  double lambda1 = getLambda();
  double lambda2 = getLambda2();
  double weight  = getWeight()(0,0);

  TaskWrapperInterface::setLambda1(lambda1);
  TaskWrapperInterface::setLambda2(lambda2);
  TaskWrapperInterface::setWeightDiag(weight);
}

void PosturalImpl::loadParams()
{

  double lambda1 = getLambda();
  double lambda2 = getLambda2();
  double weight  = getWeight()(0, 0);

  lambda1 = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".lambda1", lambda1);
  lambda2 = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".lambda2", lambda2);
  weight  = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".weight",  weight);

  // Check if the values are positive
  if(lambda1 < 0 || lambda2 < 0 || weight < 0)
    throw std::runtime_error("Lambda and weight must be positive!");

  buffer_lambda1_ = lambda1;
  buffer_lambda2_ = lambda2;
  buffer_weight_diag_ = weight;

  setLambda(lambda1,lambda2);
  setWeight(weight);
}

void PosturalImpl::updateCost(const Eigen::VectorXd& x)
{
  cost_ = computeCost(x);
}

void PosturalImpl::publish()
{
  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = "Joints";
    rt_pub_->msg_.header.stamp = task_nh_->now();

    for(unsigned int i = 0;i<getActualPositions().size();i++)
    {
      //rt_pub_->msg_.name[i] = wolf_controller::_dof_names[i]; // FIXME
      rt_pub_->msg_.position_actual[i] = getActualPositions()(i);
      rt_pub_->msg_.position_reference[i] = getReference()(i);
      rt_pub_->msg_.velocity_actual[i] =  0.0;
      rt_pub_->msg_.velocity_reference[i] = getCachedVelocityReference()(i);
      rt_pub_->msg_.position_error[i] = getError()(i);
      rt_pub_->msg_.velocity_error[i] = getVelocityError()(i);
    }

    // COST
    rt_pub_->msg_.cost = cost_;

    rt_pub_->unlockAndPublish();

  }
}

bool PosturalImpl::reset()
{
  bool res = OpenSoT::tasks::acceleration::Postural::reset(); // Task's reset

  return res;
}

void PosturalImpl::_update(const Eigen::VectorXd& x)
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_,buffer_lambda2_);
  if(OPTIONS.set_ext_weight)
    setWeight(buffer_weight_diag_);
  OpenSoT::tasks::acceleration::Postural::_update(x);
}
