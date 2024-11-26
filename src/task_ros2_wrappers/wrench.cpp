/**
 * @file wrench.cpp
 * @author Gennaro Raiola
 * @date 25 April, 2023
 * @brief This file contains the wrench task wrapper for ROS
 */

// WoLF
#include <wolf_wbid/task_ros2_wrappers/wrench.h>
#include <wolf_controller_utils/ros2_param_getter.h>

using namespace wolf_controller_utils;
using namespace wolf_wbid;

WrenchImpl::WrenchImpl(const std::string& robot_name,
                       const std::string& task_id,
                       const std::string& distal_link,
                       const std::string& base_link,
                       OpenSoT::AffineHelper& wrench,
                       const double& period)
  :Wrench(robot_name,task_id,distal_link,base_link,wrench,period)
  ,TaskRosHandler<wolf_msgs::msg::WrenchTask>(task_id,robot_name,period)
{
  tmp_vectorXd_.resize(6); // Wrench
  tmp_vectorXd_.setZero();

  // Initialize buffers
  buffer_reference_.initRT(tmp_vectorXd_);

  // Create the reference subscriber
  reference_sub_ = task_nh_->create_subscription<wolf_msgs::msg::Wrench>(
      "reference/" + _task_id, 1000,
      std::bind(&WrenchImpl::referenceCallback, this, std::placeholders::_1)
  );
}

void WrenchImpl::registerReconfigurableVariables()
{
  double lambda1 = getLambda();
  double weight  = getWeight()(0,0);

  TaskWrapperInterface::setLambda1(lambda1);
  TaskWrapperInterface::setWeightDiag(weight);
}

void WrenchImpl::loadParams()
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
}

void WrenchImpl::updateCost(const Eigen::VectorXd& x)
{
  cost_ = computeCost(x);
}

void WrenchImpl::publish()
{
  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = getBaseLink();
    rt_pub_->msg_.header.stamp = task_nh_->now();

    // FIXME
    // ACTUAL VALUES
    //getActualPose(tmp_vector3d_);
    //// Pose - Translation
    //wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.position_actual);
    //// Velocity reference
    //tmp_vector3d_ = getCachedVelocityReference();
    //wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.velocity_reference);

    // REFERENCE VALUES
    //getReference(tmp_vectorXd_);
    //// Pose - Translation
    //wolf_controller_utils::vector3dToVector3(tmp_vector3d_,rt_pub_->msg_.position_reference);

    // COST
    rt_pub_->msg_.cost = cost_;

    rt_pub_->unlockAndPublish();
  }
}

void WrenchImpl::_update(const Eigen::VectorXd& x)
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_);
  if(OPTIONS.set_ext_weight)
    setWeight(buffer_weight_diag_);
  if(OPTIONS.set_ext_reference)
  {
    // Set external reference
    setReference(*buffer_reference_.readFromRT());
  }
  OpenSoT::tasks::force::Wrench::_update(x);
}

bool WrenchImpl::reset()
{
  //bool res = OpenSoT::tasks::force::Wrench::reset(); // Task's reset (FIXME it is not implemented in OpenSoT)
  bool res = true;
  getReference(tmp_vectorXd_);
  buffer_reference_.initRT(tmp_vectorXd_);
  //getActualPose(tmp_vector3d_);
  //tmp_affine3d_ = Eigen::Affine3d::Identity();
  //tmp_affine3d_.translation() = tmp_vector3d_;
  //trj_->reset(tmp_affine3d_);
  return res;
}

void WrenchImpl::referenceCallback(const wolf_msgs::msg::Wrench::SharedPtr msg)
{
  double period = period_;

  if (last_time_ != 0.0)
      period = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9 - last_time_;

  Eigen::Vector6d reference = Eigen::Vector6d::Zero();
  reference(0) = msg->wrench.force.x;
  reference(1) = msg->wrench.force.y;
  reference(2) = msg->wrench.force.z;
  buffer_reference_.writeFromNonRT(reference);

  last_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
}
