/**
 * @file wrench.cpp
 * @author Gennaro Raiola
 * @brief ROS wrapper for OpenSoT-free POINT_CONTACT wrench task
 */

#include <wolf_wbid/ros/wrench.h>

// STD
#include <stdexcept>

namespace wolf_wbid {

WrenchImpl::WrenchImpl(const std::string& robot_name,
                       const std::string& task_id,
                       const std::string& contact_name,
                       const IDVariables& vars,
                       const double& period)
  : Wrench(robot_name, task_id, contact_name, vars, period, 1.0)
  , TaskRosHandler<wolf_msgs::WrenchTask>(task_id, robot_name, period)
  , vars_(vars)
{
  buffer_reference_force_.initRT(Eigen::Vector3d::Zero());

  // reference subscriber
  reference_sub_ = nh_.subscribe("reference/" + task_id, 1000,
                                 &WrenchImpl::referenceCallback, this);

  // init buffer from task
  buffer_weight_diag_ = this->weight();
}

void WrenchImpl::registerReconfigurableVariables()
{
  const double w = this->weight();

  ddr_server_->registerVariable<double>(
        "set_weight_diag", w,
        boost::bind(&TaskWrapperInterface::setWeightDiag, this, _1),
        "set weight diag (scalar)", 0.0, 10000.0);

  ddr_server_->publishServicesTopics();
}

void WrenchImpl::loadParams()
{
  double weight;

  if(!nh_.getParam("gains/" + task_name_ + "/weight", weight))
    weight = this->weight();

  if(weight < 0.0)
    throw std::runtime_error("WrenchImpl::loadParams(): weight must be >= 0");

  buffer_weight_diag_ = weight;

  // apply immediately
  this->setWeight(weight);
}

void WrenchImpl::applyExternalKnobs()
{
  if(OPTIONS.set_ext_weight)
    this->setWeight(buffer_weight_diag_.load());

  // no lambda / gains here (unless you add them in WrenchTask later)
}

void WrenchImpl::applyExternalReference()
{
  if(OPTIONS.set_ext_reference)
    this->setReference(*buffer_reference_force_.readFromRT());
}

void WrenchImpl::updateCost(const Eigen::VectorXd& x)
{
  // Option A (preferred): if WrenchTask can compute actual force from x internally, use that.
  // Otherwise, we can extract from x using IDVariables mapping (Option B).

  // ---------------------------
  // Option B: extract from x (requires stable IDVariables API)
  // ---------------------------
  // EXPECTED API (you likely already have something like this):
  //   int contactOffset(const std::string& name) const;
  // If you have a different name, I’ll align it.

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
    rt_pub_->msg_.header.stamp = ros::Time::now();

    const Eigen::Vector3d f_ref = this->reference();

    // reference
    rt_pub_->msg_.wrench_reference.force.x  = f_ref.x();
    rt_pub_->msg_.wrench_reference.force.y  = f_ref.y();
    rt_pub_->msg_.wrench_reference.force.z  = f_ref.z();
    rt_pub_->msg_.wrench_reference.torque.x = 0.0;
    rt_pub_->msg_.wrench_reference.torque.y = 0.0;
    rt_pub_->msg_.wrench_reference.torque.z = 0.0;

    // actual (if we cached it from updateCost)
    if(has_last_f_act_)
    {
      rt_pub_->msg_.wrench_actual.force.x  = last_f_act_.x();
      rt_pub_->msg_.wrench_actual.force.y  = last_f_act_.y();
      rt_pub_->msg_.wrench_actual.force.z  = last_f_act_.z();
    }
    else
    {
      rt_pub_->msg_.wrench_actual.force.x  = 0.0;
      rt_pub_->msg_.wrench_actual.force.y  = 0.0;
      rt_pub_->msg_.wrench_actual.force.z  = 0.0;
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

  buffer_weight_diag_ = this->weight();
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
