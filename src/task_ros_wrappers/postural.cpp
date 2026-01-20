#include <wolf_wbid/task_ros_wrappers/postural.h>

#include <stdexcept>

namespace wolf_wbid {

PosturalImpl::PosturalImpl(const std::string& robot_name,
                           const std::string& task_id,
                           QuadrupedRobot& robot,
                           const IDVariables& idvars,
                           double period)
: Postural(robot_name, robot, idvars, task_id, period)
, TaskRosHandler<wolf_msgs::PosturalTask>(task_id, robot_name, period)
{
  // Pre-size msg arrays (publish() also resizes defensively)
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

  // Init wrapper buffers with current task params
  buffer_lambda1_     = getLambda1();
  buffer_lambda2_     = getLambda2();
  buffer_weight_diag_ = getWeightDiag();
}

void PosturalImpl::registerReconfigurableVariables()
{
  const double lambda1 = getLambda1();
  const double lambda2 = getLambda2();
  const double weight  = getWeightDiag();

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
    "set weight diag", 0.0, 10000.0);

  ddr_server_->publishServicesTopics();
}

void PosturalImpl::loadParams()
{
  double lambda1, lambda2, weight;

  // Use TaskWrapperInterface::task_name_ consistently
  if(!nh_.getParam("gains/" + task_name_ + "/lambda1", lambda1))
    lambda1 = getLambda1();

  if(!nh_.getParam("gains/" + task_name_ + "/lambda2", lambda2))
    lambda2 = getLambda2();

  if(!nh_.getParam("gains/" + task_name_ + "/weight", weight))
    weight = getWeightDiag();

  if(lambda1 < 0.0 || lambda2 < 0.0 || weight < 0.0)
    throw std::runtime_error("PosturalImpl::loadParams(): lambda/weight must be >= 0");

  buffer_lambda1_     = lambda1;
  buffer_lambda2_     = lambda2;
  buffer_weight_diag_ = weight;

  // Push immediately at startup
  setLambda(lambda1, lambda2);
  PosturalTask::setWeightDiag(weight);
}

void PosturalImpl::applyExternalKnobs()
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_.load(), buffer_lambda2_.load());

  if(OPTIONS.set_ext_weight)
    PosturalTask::setWeightDiag(buffer_weight_diag_.load());

  // Postural gains (Kp/Kd) are not exposed here in your micro interface.
  // If/when you add them in PosturalTask, plug them here like Com/Cartesian.
}

void PosturalImpl::applyExternalReference()
{
  // Currently no external reference topic for postural.
  // If you add one, write buffers here and call PosturalTask::setReference(qref, qdref).
}

void PosturalImpl::updateCost(const Eigen::VectorXd& x)
{
  // If PosturalTask has computeCost(x) keep it; otherwise use generic (Ax-b)'W(Ax-b)
  cost_ = computeCost(x);
}

bool PosturalImpl::reset()
{
  const bool ok = PosturalTask::reset();

  // Sync wrapper buffers with current task params
  buffer_lambda1_     = getLambda1();
  buffer_lambda2_     = getLambda2();
  buffer_weight_diag_ = getWeightDiag();

  return ok;
}

void PosturalImpl::publish()
{
  if(!rt_pub_) return;

  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = "Joints";
    rt_pub_->msg_.header.stamp    = ros::Time::now();

    const auto& names = jointNames();
    const auto& q     = actualQ();
    const auto& qdot  = actualQdot();
    const auto& qref  = refQ();
    const auto& qdref = refQdotCached();
    const auto& e     = posError();
    const auto& edot  = velError();

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

      rt_pub_->msg_.position_actual[i]    = q(i);
      rt_pub_->msg_.position_reference[i] = qref(i);

      rt_pub_->msg_.velocity_actual[i]    = qdot(i);
      rt_pub_->msg_.velocity_reference[i] = (qdref.size() == qdot.size()) ? qdref(i) : 0.0;

      rt_pub_->msg_.position_error[i] = e(i);
      rt_pub_->msg_.velocity_error[i] = edot(i);
    }

    rt_pub_->msg_.cost = cost_;

    rt_pub_->unlockAndPublish();
  }
}

} // namespace wolf_wbid
