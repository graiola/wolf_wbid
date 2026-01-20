// ============================================================================
// File: src/task_ros_wrappers/momentum.cpp
// ============================================================================

/**
 * @file momentum.cpp
 * @author Gennaro Raiola
 * @brief ROS wrapper for OpenSoT-free AngularMomentum task
 */

#include <wolf_wbid/task_ros_wrappers/momentum.h>
#include <stdexcept>

using namespace wolf_wbid;

AngularMomentumImpl::AngularMomentumImpl(const std::string& robot_name,
                                         QuadrupedRobot& robot,
                                         const IDVariables& vars,
                                         const std::string& task_id,
                                         const double& period)
  : AngularMomentum(robot_name, robot, vars, task_id, period)
  , TaskRosHandler<wolf_msgs::CartesianTask>(task_id, robot_name, period)
{
}

void AngularMomentumImpl::registerReconfigurableVariables()
{
  const double lambda1 = getLambda();
  const double weight  = getWeight()(0,0);

  ddr_server_->registerVariable<double>(
        "set_lambda_1", lambda1,
        boost::bind(&TaskWrapperInterface::setLambda1, this, _1),
        "set lambda 1", 0.0, 1000.0);

  ddr_server_->registerVariable<double>(
        "set_weight_diag", weight,
        boost::bind(&TaskWrapperInterface::setWeightDiag, this, _1),
        "set weight diag", 0.0, 1000.0);

  Eigen::Matrix3d K = getMomentumGain();
  ddr_server_->registerVariable<double>("K_roll",  K(0,0), boost::bind(&TaskWrapperInterface::setKpRoll,  this,_1), "K(0,0)", 0.0, 1000.0);
  ddr_server_->registerVariable<double>("K_pitch", K(1,1), boost::bind(&TaskWrapperInterface::setKpPitch, this,_1), "K(1,1)", 0.0, 1000.0);
  ddr_server_->registerVariable<double>("K_yaw",   K(2,2), boost::bind(&TaskWrapperInterface::setKpYaw,   this,_1), "K(2,2)", 0.0, 1000.0);

  ddr_server_->publishServicesTopics();
}

void AngularMomentumImpl::loadParams()
{
  double lambda1, weight;

  if(!nh_.getParam("gains/" + task_name_ + "/lambda1", lambda1))
    lambda1 = getLambda();

  if(!nh_.getParam("gains/" + task_name_ + "/weight", weight))
    weight = getWeight()(0,0);

  if(lambda1 < 0.0 || weight < 0.0)
    throw std::runtime_error("AngularMomentumImpl::loadParams(): lambda/weight must be >= 0");

  buffer_lambda1_ = lambda1;
  buffer_weight_diag_ = weight;

  setLambda(lambda1);

  Eigen::Matrix3d Wd = Eigen::Matrix3d::Identity() * weight;
  setWeight(Wd);

  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  bool use_identity = false;

  for(unsigned int i=0; i<wolf_controller_utils::_rpy.size(); i++)
  {
    if(!nh_.getParam("gains/" + task_name_ + "/K/" + wolf_controller_utils::_rpy[i], K(i,i)))
      use_identity = true;

    if(K(i,i) < 0.0)
      use_identity = true;
  }

  if(use_identity)
    K = Eigen::Matrix3d::Identity();

  buffer_kp_roll_  = K(0,0);
  buffer_kp_pitch_ = K(1,1);
  buffer_kp_yaw_   = K(2,2);

  setMomentumGain(K);
}

void AngularMomentumImpl::updateCost(const Eigen::VectorXd& x)
{
  const Eigen::VectorXd r = A() * x - b();
  cost_ = 0.5 * (r.transpose() * W() * r)(0,0);
}

void AngularMomentumImpl::update(const Eigen::VectorXd& x)
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_.load());

  if(OPTIONS.set_ext_weight)
  {
    const double w = buffer_weight_diag_.load();
    Eigen::Matrix3d Wd = Eigen::Matrix3d::Identity() * w;
    setWeight(Wd);
  }

  if(OPTIONS.set_ext_gains)
  {
    tmp_matrix3d_.setZero();
    tmp_matrix3d_(0,0) = buffer_kp_roll_.load();
    tmp_matrix3d_(1,1) = buffer_kp_pitch_.load();
    tmp_matrix3d_(2,2) = buffer_kp_yaw_.load();
    setMomentumGain(tmp_matrix3d_);
  }

  AngularMomentumTask::update(x);
}

void AngularMomentumImpl::publish()
{
  if(!rt_pub_) return;

  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = getBaseLink();
    rt_pub_->msg_.header.stamp = ros::Time::now();

    // Reuse CartesianTask message fields for angular momentum:
    // - rpy_actual    := actual angular momentum L
    // - rpy_reference := desired angular momentum (cached)
    Eigen::Vector3d L_act = getActualAngularMomentum();
    Eigen::Vector3d L_ref = getCachedDesiredAngularMomentum();

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
