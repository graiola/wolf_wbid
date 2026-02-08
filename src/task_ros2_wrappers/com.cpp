/**
 * @file com.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the com task wrapper for ROS2
 */

// WoLF
#include <wolf_wbid/task_ros2_wrappers/com.h>
#include <wolf_controller_utils/ros2_param_getter.h>

using namespace wolf_controller_utils;
using namespace wolf_wbid;

ComImpl::ComImpl(const std::string& robot_name,
                 QuadrupedRobot& robot,
                 const OpenSoT::AffineHelper& qddot,
                 const double& period)
: Com(robot_name, robot, qddot, period),
  TaskRosHandler<wolf_msgs::msg::ComTask>(_task_id,robot_name, period)
{
    tmp_vector3d_.setZero();
    buffer_reference_pos_.initRT(tmp_vector3d_);
    buffer_reference_vel_.initRT(tmp_vector3d_);

    reference_sub_ = task_nh_->create_subscription<wolf_msgs::msg::Com>(
        "reference/" + _task_id, 1000,
        std::bind(&ComImpl::referenceCallback, this, std::placeholders::_1)
    );
}

void ComImpl::registerReconfigurableVariables()
{
    double lambda1 = getLambda();
    double lambda2 = getLambda2();
    double weight  = getWeight()(0,0);
    Eigen::Matrix3d Kp = getKp();
    Eigen::Matrix3d Kd = getKd();

    TaskWrapperInterface::setLambda1(lambda1);
    TaskWrapperInterface::setLambda2(lambda2);
    TaskWrapperInterface::setWeightDiag(weight);

    TaskWrapperInterface::setKpX(Kp(0,0));
    TaskWrapperInterface::setKpY(Kp(1,1));
    TaskWrapperInterface::setKpZ(Kp(2,2));

    TaskWrapperInterface::setKdX(Kd(0,0));
    TaskWrapperInterface::setKdY(Kd(1,1));
    TaskWrapperInterface::setKdZ(Kd(2,2));
}

void ComImpl::loadParams()
{
    double lambda1 = getLambda();
    double lambda2 = getLambda2();
    double weight  = getWeight()(0, 0);

    lambda1 = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".lambda1", lambda1);
    lambda2 = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".lambda2", lambda2);
    weight  = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".weight",  weight);

    if(lambda1 < 0 || lambda2 < 0 || weight < 0)
        throw std::runtime_error("Lambda and weight must be positive!");

    buffer_lambda1_ = lambda1;
    buffer_lambda2_ = lambda2;
    buffer_weight_diag_ = weight;

    setLambda(lambda1, lambda2);
    setWeight(weight);

    Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Kd = Eigen::Matrix3d::Zero();
    bool use_identity = false;

    for(unsigned int i=0; i<wolf_controller_utils::_xyz.size(); i++)
    {

        Kp(i, i) = get_double_parameter_from_remote_node("wolf_controller/gains." + _task_id + ".Kp." + wolf_controller_utils::_xyz[i], 0.0);
        Kd(i, i) = get_double_parameter_from_remote_node("wolf_controller/gains." + _task_id + ".Kd." + wolf_controller_utils::_xyz[i], 0.0);

        // Check if the values are positive
        if(Kp(i,i)<0.0 || Kd(i,i)<0.0)
          use_identity = true;
    }

    if(use_identity)
    {
        Kp = Eigen::Matrix3d::Identity();
        Kd = Eigen::Matrix3d::Identity();
    }

    buffer_kp_x_     = Kp(0,0);
    buffer_kp_y_     = Kp(1,1);
    buffer_kp_z_     = Kp(2,2);

    buffer_kd_x_     = Kd(0,0);
    buffer_kd_y_     = Kd(1,1);
    buffer_kd_z_     = Kd(2,2);

    setKp(Kp);
    setKd(Kd);
}

void ComImpl::updateCost(const Eigen::VectorXd& x)
{
    cost_ = computeCost(x);
}

void ComImpl::publish()
{
    if (rt_pub_->trylock())
    {
        rt_pub_->msg_.header.frame_id = getBaseLink();
        rt_pub_->msg_.header.stamp = task_nh_->now();

        getActualPose(tmp_vector3d_);
        wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.position_actual);

        tmp_vector3d_ = getCachedVelocityReference();
        wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.velocity_reference);

        getReference(tmp_vector3d_);
        wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.position_reference);

        rt_pub_->msg_.cost = cost_;

        rt_pub_->unlockAndPublish();
    }
}

bool ComImpl::reset()
{
    bool res = OpenSoT::tasks::acceleration::CoM::reset();
    getActualPose(tmp_vector3d_);
    buffer_reference_pos_.initRT(tmp_vector3d_);
    return res;
}

void ComImpl::_update(const Eigen::VectorXd& x)
{
    if (OPTIONS.set_ext_lambda)
        setLambda(buffer_lambda1_, buffer_lambda2_);
    if (OPTIONS.set_ext_weight)
        setWeight(buffer_weight_diag_);
    if (OPTIONS.set_ext_gains)
    {
        Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero();
        Kp(0,0) = buffer_kp_x_;
        Kp(1,1) = buffer_kp_y_;
        Kp(2,2) = buffer_kp_z_;
        setKp(Kp);

        Eigen::Matrix3d Kd = Eigen::Matrix3d::Zero();
        Kd(0,0) = buffer_kd_x_;
        Kd(1,1) = buffer_kd_y_;
        Kd(2,2) = buffer_kd_z_;
        setKd(Kd);
    }
    if (OPTIONS.set_ext_reference)
    {
        setReference(*buffer_reference_pos_.readFromRT(), *buffer_reference_vel_.readFromRT());
    }
    OpenSoT::tasks::acceleration::CoM::_update(x);
}

void ComImpl::referenceCallback(const wolf_msgs::msg::Com::SharedPtr msg)
{
    double period = period_;

    if (last_time_ != 0.0)
        period = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9 - last_time_;

    Eigen::Vector3d position_reference = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_reference = Eigen::Vector3d::Zero();

    position_reference.x() = msg->position.x;
    position_reference.y() = msg->position.y;
    position_reference.z() = msg->position.z;

    velocity_reference.x() = msg->velocity.x;
    velocity_reference.y() = msg->velocity.y;
    velocity_reference.z() = msg->velocity.z;

    buffer_reference_pos_.writeFromNonRT(position_reference);
    buffer_reference_vel_.writeFromNonRT(velocity_reference);

    last_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
}
