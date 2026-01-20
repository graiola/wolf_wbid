#include <wolf_wbid/task_ros_wrappers/postural.h>

namespace wolf_wbid {

PosturalImpl::PosturalImpl(const std::string& robot_name,
                           const std::string& task_id,
                           QuadrupedRobot& robot,
                           const IDVariables& idvars,
                           double period)
: Postural(robot_name, robot, idvars, task_id, period)
, TaskRosHandler<wolf_msgs::PosturalTask>(task_id, robot_name, period)
{
  // pre-size msg arrays (safe; publish() also resizes if needed)
  const std::size_t n = static_cast<std::size_t>(taskSize());
  rt_pub_->msg_.name.resize(n);
  rt_pub_->msg_.position_actual.resize(n);
  rt_pub_->msg_.position_reference.resize(n);
  rt_pub_->msg_.velocity_actual.resize(n);
  rt_pub_->msg_.velocity_reference.resize(n);
  rt_pub_->msg_.position_error.resize(n);
  rt_pub_->msg_.velocity_error.resize(n);

  // defaults for wrapper buffers (optional)
  buffer_lambda1_ = getLambda1();
  buffer_lambda2_ = getLambda2();
  buffer_weight_diag_ = getWeightDiag();
}

void PosturalImpl::registerReconfigurableVariables()
{
  double lambda1 = getLambda1();
  double lambda2 = getLambda2();
  double weight  = getWeightDiag();

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

  ddr_server_->publishServicesTopics();
}

void PosturalImpl::loadParams()
{
  double lambda1, lambda2, weight;

  if(!nh_.getParam("gains/" + id() + "/lambda1", lambda1))
    lambda1 = getLambda1();

  if(!nh_.getParam("gains/" + id() + "/lambda2", lambda2))
    lambda2 = getLambda2(); // oppure 2*sqrt(lambda1)

  if(!nh_.getParam("gains/" + id() + "/weight", weight))
    weight = getWeightDiag();

  if(lambda1 < 0 || lambda2 < 0 || weight < 0)
    throw std::runtime_error("PosturalImpl: Lambda and weight must be positive!");

  buffer_lambda1_ = lambda1;
  buffer_lambda2_ = lambda2;
  buffer_weight_diag_ = weight;

  setLambda(lambda1, lambda2);
  PosturalTask::setWeightDiag(weight);
}

void PosturalImpl::updateCost(const Eigen::VectorXd& x)
{
  cost_ = computeCost(x);
}

bool PosturalImpl::reset()
{
  // reset core + keep wrapper buffers consistent
  const bool ok = PosturalTask::reset();
  buffer_lambda1_ = getLambda1();
  buffer_lambda2_ = getLambda2();
  buffer_weight_diag_ = getWeightDiag();
  return ok;
}

void PosturalImpl::_update(const Eigen::VectorXd& x)
{
  // apply buffered external params
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_, buffer_lambda2_);
  if(OPTIONS.set_ext_weight)
    PosturalTask::setWeightDiag(buffer_weight_diag_.load());

  // now run core update
  PosturalTask::_update(x);
}

void PosturalImpl::publish()
{
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

    rt_pub_->msg_.cost = cost_; // oppure costLast()

    rt_pub_->unlockAndPublish();
  }
}

} // namespace wolf_wbid
