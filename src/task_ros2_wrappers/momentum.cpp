/**
 * @file momentum.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the momentum task wrapper for ROS
 */

// WoLF
#include <wolf_wbid/task_ros2_wrappers/momentum.h>

#include <wolf_controller_utils/ros2_param_getter.h>

using namespace wolf_controller_utils;
using namespace wolf_wbid;

AngularMomentumImpl::AngularMomentumImpl(const std::string& robot_name, XBot::ModelInterface& robot, const OpenSoT::AffineHelper& qddot, const double& period)
  :AngularMomentum(robot_name,robot,qddot,period)
  ,TaskRosHandler<wolf_msgs::msg::CartesianTask>(_task_id,robot_name,period)
{
}

void AngularMomentumImpl::registerReconfigurableVariables()
{
  double lambda1 = getLambda();
  double weight  = getWeight()(0,0);
  Eigen::Matrix3d K = getMomentumGain();

  // Load the tmp variables used in _update
  TaskWrapperInterface::setLambda1(lambda1);
  TaskWrapperInterface::setWeightDiag(weight);

  TaskWrapperInterface::setKpRoll(K(0,0));
  TaskWrapperInterface::setKpPitch(K(1,1));
  TaskWrapperInterface::setKpYaw(K(2,2));

}

void AngularMomentumImpl::loadParams()
{

  double lambda1 = getLambda();
  double weight  = getWeight()(0, 0);

  lambda1 = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".lambda1", lambda1);
  weight  = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".weight",  weight);

  // Check if the values are positive
  if(lambda1 < 0 || weight < 0)
    throw std::runtime_error("Lambda and weight must be positive!");

  buffer_lambda1_ = lambda1;
  buffer_weight_diag_ = weight;

  setLambda(lambda1);
  setWeight(weight);

  // Load params
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

  bool use_identity = false;
  for(unsigned int i=0; i<wolf_controller_utils::_rpy.size(); i++)
  {

    K(i,i) = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".K." + wolf_controller_utils::_rpy[i], K(i,i));

    if(K(i,i)<0.0)
      use_identity = true;
  }

  if(use_identity)
    K = Eigen::Matrix3d::Identity();

  buffer_kp_roll_   = K(0,0);
  buffer_kp_pitch_  = K(1,1);
  buffer_kp_yaw_    = K(2,2);

  setMomentumGain(K);
}

void AngularMomentumImpl::updateCost(const Eigen::VectorXd& x)
{
  cost_ = computeCost(x);
}

void AngularMomentumImpl::update(const Eigen::VectorXd& x)
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_);
  if(OPTIONS.set_ext_weight)
    setWeight(buffer_weight_diag_);
  if(OPTIONS.set_ext_gains)
  {
    tmp_matrix3d_.setZero();
    tmp_matrix3d_(0,0) = buffer_kp_roll_;
    tmp_matrix3d_(1,1) = buffer_kp_pitch_;
    tmp_matrix3d_(2,2) = buffer_kp_yaw_;
    setMomentumGain(tmp_matrix3d_);
  }
  OpenSoT::tasks::acceleration::AngularMomentum::update(x);
}

void AngularMomentumImpl::publish()
{

}

bool AngularMomentumImpl::reset()
{
  bool res = OpenSoT::tasks::acceleration::AngularMomentum::reset(); // Task's reset

  return res;
}
