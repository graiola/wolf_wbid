/**
 * @file com.cpp
 * @author Gennaro Raiola
 * @brief ROS wrapper for OpenSoT-free CoM task
 */

#include <wolf_wbid/task_ros_wrappers/com.h>

#include <wolf_controller_utils/converters.h>

// STD
#include <stdexcept>

namespace wolf_wbid {

ComImpl::ComImpl(const std::string& robot_name,
                 const std::string& task_id, 
                 QuadrupedRobot& robot,
                 const IDVariables& vars,
                 const double& period)
  : Com(robot_name, task_id, robot, vars, period)
  , TaskRosHandler<wolf_msgs::ComTask>(task_id, robot_name, period)
{
  // Init realtime buffers
  tmp_vector3d_.setZero();
  buffer_reference_pos_.initRT(tmp_vector3d_);
  buffer_reference_vel_.initRT(tmp_vector3d_);

  // Create reference subscriber
  reference_sub_ = nh_.subscribe("reference/" + task_id, 1000, &ComImpl::referenceCallback, this);
}

void ComImpl::registerReconfigurableVariables()
{
  const double lambda1 = getLambda();
  const double lambda2 = getLambda2();
  const double weight  = getWeight()(0,0);  // diagonal assumption in UI

  ddr_server_->registerVariable<double>(
        "set_lambda_1", lambda1,
        boost::bind(&TaskWrapperInterface::setLambda1, this, _1),
        "set lambda 1", 0.0, 1000.0);

  ddr_server_->registerVariable<double>(
        "set_lambda_2", lambda2,
        boost::bind(&TaskWrapperInterface::setLambda2, this, _1),
        "set lambda 2", 0.0, 1000.0);

  ddr_server_->registerVariable<double>(
        "set_weight_diag", weight,
        boost::bind(&TaskWrapperInterface::setWeightDiag, this, _1),
        "set weight diag", 0.0, 1000.0);

  Eigen::Matrix3d Kp = getKp();
  ddr_server_->registerVariable<double>("kp_x", Kp(0,0), boost::bind(&TaskWrapperInterface::setKpX,this,_1), "Kp(0,0)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_y", Kp(1,1), boost::bind(&TaskWrapperInterface::setKpY,this,_1), "Kp(1,1)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_z", Kp(2,2), boost::bind(&TaskWrapperInterface::setKpZ,this,_1), "Kp(2,2)", 0.0, 10000.0);

  Eigen::Matrix3d Kd = getKd();
  ddr_server_->registerVariable<double>("kd_x", Kd(0,0), boost::bind(&TaskWrapperInterface::setKdX,this,_1), "Kd(0,0)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_y", Kd(1,1), boost::bind(&TaskWrapperInterface::setKdY,this,_1), "Kd(1,1)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_z", Kd(2,2), boost::bind(&TaskWrapperInterface::setKdZ,this,_1), "Kd(2,2)", 0.0, 10000.0);

  ddr_server_->publishServicesTopics();
}

void ComImpl::loadParams()
{
  double lambda1, lambda2, weight;

  // NOTE: use task_name_ (from TaskWrapperInterface) for param namespace consistency
  if(!nh_.getParam("gains/" + task_name_ + "/lambda1", lambda1)) lambda1 = getLambda();
  if(!nh_.getParam("gains/" + task_name_ + "/lambda2", lambda2)) lambda2 = getLambda2();
  if(!nh_.getParam("gains/" + task_name_ + "/weight",  weight))  weight  = getWeight()(0,0);

  if(lambda1 < 0 || lambda2 < 0 || weight < 0)
    throw std::runtime_error("ComImpl::loadParams(): Lambda and weight must be positive!");

  buffer_lambda1_     = lambda1;
  buffer_lambda2_     = lambda2;
  buffer_weight_diag_ = weight;

  setLambda(lambda1, lambda2);
  setWeight(Eigen::Matrix3d::Identity() * weight);

  Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d Kd = Eigen::Matrix3d::Zero();
  bool use_identity = false;

  for(unsigned int i = 0; i < wolf_controller_utils::_xyz.size(); i++)
  {
    if(!nh_.getParam("gains/" + task_name_ + "/Kp/" + wolf_controller_utils::_xyz[i], Kp(i,i))) use_identity = true;
    if(!nh_.getParam("gains/" + task_name_ + "/Kd/" + wolf_controller_utils::_xyz[i], Kd(i,i))) use_identity = true;

    if(Kp(i,i) < 0.0 || Kd(i,i) < 0.0)
      throw std::runtime_error("ComImpl::loadParams(): Kp and Kd must be positive!");
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

void ComImpl::update(const Eigen::VectorXd& x)
{
  // apply external knobs (like your old OpenSoT _update wrapper)

  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_.load(), buffer_lambda2_.load());

  if(OPTIONS.set_ext_weight)
    setWeight(Eigen::Matrix3d::Identity() * buffer_weight_diag_.load());

  if(OPTIONS.set_ext_gains)
  {
    tmp_matrix3d_.setZero();
    tmp_matrix3d_(0,0) = buffer_kp_x_.load();
    tmp_matrix3d_(1,1) = buffer_kp_y_.load();
    tmp_matrix3d_(2,2) = buffer_kp_z_.load();
    setKp(tmp_matrix3d_);

    tmp_matrix3d_.setZero();
    tmp_matrix3d_(0,0) = buffer_kd_x_.load();
    tmp_matrix3d_(1,1) = buffer_kd_y_.load();
    tmp_matrix3d_(2,2) = buffer_kd_z_.load();
    setKd(tmp_matrix3d_);
  }

  if(OPTIONS.set_ext_reference)
  {
    setReference(*buffer_reference_pos_.readFromRT(),
                 *buffer_reference_vel_.readFromRT());
  }

  // run task math
  ComTask::update(x);
}

void ComImpl::updateCost(const Eigen::VectorXd& x)
{
  // cost = 0.5 * (Ax-b)^T W (Ax-b)
  const Eigen::VectorXd r = A() * x - b();
  cost_ = 0.5 * (r.transpose() * W() * r)(0,0);
}

void ComImpl::publish()
{
  if(!rt_pub_) return;

  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = getBaseLink();
    rt_pub_->msg_.header.stamp = ros::Time::now();

    // ACTUAL position + velocity (valid after update())
    getActualPose(tmp_vector3d_);
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.position_actual);

    getActualVelocity(tmp_vector3d_);
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.velocity_actual);

    // REFERENCE position
    getReference(tmp_vector3d_);
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.position_reference);

    // REFERENCE velocity (cached, OpenSoT-like)
    tmp_vector3d_ = getCachedVelocityReference();
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.velocity_reference);

    // COST
    rt_pub_->msg_.cost = cost_;

    rt_pub_->unlockAndPublish();
  }
}

bool ComImpl::reset()
{
  const bool ok = ComTask::reset();

  // sync RT buffers with current actual CoM
  getActualPose(tmp_vector3d_);
  buffer_reference_pos_.initRT(tmp_vector3d_);

  tmp_vector3d_.setZero();
  buffer_reference_vel_.initRT(tmp_vector3d_);

  return ok;
}

void ComImpl::referenceCallback(const wolf_msgs::Com::ConstPtr& msg)
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

  last_time_ = msg->header.stamp.toSec();
}

} // namespace wolf_wbid
