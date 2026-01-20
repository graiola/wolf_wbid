/**
 * @file wrench.cpp
 * @author Gennaro Raiola
 * @brief ROS wrapper for OpenSoT-free POINT_CONTACT wrench task
 */

#include <wolf_wbid/task_ros_wrappers/wrench.h>

// STD
#include <stdexcept>

namespace wolf_wbid {

WrenchImpl::WrenchImpl(const std::string& robot_name,
                       const std::string& task_id,
                       const std::string& contact_name,
                       const IDVariables& vars,
                       const double& period)
  : Wrench(robot_name, task_id, contact_name, period)
  , TaskRosHandler<wolf_msgs::WrenchTask>(task_id, robot_name, period)
  , vars_(vars)
{
  Eigen::Vector3d z = Eigen::Vector3d::Zero();
  buffer_reference_force_.initRT(z);

  // reference subscriber
  reference_sub_ = nh_.subscribe("reference/" + task_id, 1000,
                                 &WrenchImpl::referenceCallback, this);
}

void WrenchImpl::registerReconfigurableVariables()
{
  // Only weight for this task (no lambda in OpenSoT-free wrench task)
  double w = this->weight();

  ddr_server_->registerVariable<double>(
        "set_weight_diag", w,
        boost::bind(&TaskWrapperInterface::setWeightDiag, this, _1),
        "set weight diag (scalar)", 0.0, 10000.0);

  ddr_server_->publishServicesTopics();
}

void WrenchImpl::loadParams()
{
  double weight;

  // IMPORTANT: avoid ambiguity by explicitly using TaskWrapperInterface::task_name_
  const std::string& tn = TaskWrapperInterface::task_name_;

  if(!nh_.getParam("gains/" + tn + "/weight", weight))
    weight = this->weight();

  if(weight < 0.0)
    throw std::runtime_error("WrenchImpl::loadParams(): weight must be positive!");

  buffer_weight_diag_ = weight;

  // apply
  this->setWeight(weight);
}

void WrenchImpl::update(const Eigen::VectorXd& /*x*/)
{
  // dynamic reconfigure -> task params
  if(OPTIONS.set_ext_weight)
  {
    this->setWeight(buffer_weight_diag_.load());
  }

  // external reference (topic)
  if(OPTIONS.set_ext_reference)
  {
    this->setReference(*buffer_reference_force_.readFromRT());
  }

  // NOTE: No math here: the QP term is built later by calling WrenchTask::compute(vars, term)
}

void WrenchImpl::updateCost(const Eigen::VectorXd& x)
{
  // cost = 0.5 * w * ||f - fref||^2
  if(vars_.contactDim() != 3)
    throw std::runtime_error("WrenchImpl::updateCost(): only POINT_CONTACT supported");

  const int off = vars_.contactOffset(this->contactName());
  if(off < 0 || off + 3 > x.size())
    throw std::runtime_error("WrenchImpl::updateCost(): contact block out of range");

  const Eigen::Vector3d f_act = x.segment<3>(off);
  const Eigen::Vector3d f_ref = this->reference();
  const Eigen::Vector3d e = f_act - f_ref;

  cost_ = 0.5 * this->weight() * e.squaredNorm();
}

void WrenchImpl::publish()
{
  if(!rt_pub_) return;

  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = WORLD_FRAME_NAME; // or keep as task base if you want
    rt_pub_->msg_.header.stamp = ros::Time::now();

    // actual from last solution is not stored in task: compute from last x_ if you have it,
    // here we publish what we can: reference + cost. If you want actual, pass x_ into publish().
    // For now publish reference as the desired wrench (force only), actual zeros.
    const Eigen::Vector3d f_ref = this->reference();

    // reference
    rt_pub_->msg_.wrench_reference.force.x  = f_ref.x();
    rt_pub_->msg_.wrench_reference.force.y  = f_ref.y();
    rt_pub_->msg_.wrench_reference.force.z  = f_ref.z();
    rt_pub_->msg_.wrench_reference.torque.x = 0.0;
    rt_pub_->msg_.wrench_reference.torque.y = 0.0;
    rt_pub_->msg_.wrench_reference.torque.z = 0.0;

    // actual (not available here without x) => set to 0
    rt_pub_->msg_.wrench_actual.force.x  = 0.0;
    rt_pub_->msg_.wrench_actual.force.y  = 0.0;
    rt_pub_->msg_.wrench_actual.force.z  = 0.0;
    rt_pub_->msg_.wrench_actual.torque.x = 0.0;
    rt_pub_->msg_.wrench_actual.torque.y = 0.0;
    rt_pub_->msg_.wrench_actual.torque.z = 0.0;

    rt_pub_->msg_.cost = cost_;

    rt_pub_->unlockAndPublish();
  }
}

bool WrenchImpl::reset()
{
  // reset reference to zero (task-level)
  this->WrenchTask::reset();

  // sync RT buffer
  Eigen::Vector3d z = Eigen::Vector3d::Zero();
  buffer_reference_force_.initRT(z);

  return true;
}

void WrenchImpl::referenceCallback(const wolf_msgs::Wrench::ConstPtr& msg)
{
  Eigen::Vector3d f;
  f.x() = msg->wrench.force.x;
  f.y() = msg->wrench.force.y;
  f.z() = msg->wrench.force.z;

  buffer_reference_force_.writeFromNonRT(f);
  last_time_ = msg->header.stamp.toSec();
}

} // namespace wolf_wbid
