/**
 * @file com.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the com task wrapper for ROS2
 */

// WoLF
#include <wolf_wbid/task_ros2_wrappers/com.h>

using namespace wolf_wbid;

ComImpl::ComImpl(const std::string& robot_name,
                 const XBot::ModelInterface& robot,
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

    // FIXME
    /*ddr_server_->register_variable("set_lambda_1", lambda1, [this](double val) { setLambda1(val); }, "set lambda 1", 0.0, 1000.0);
    ddr_server_->register_variable("set_lambda_2", lambda2, [this](double val) { setLambda2(val); }, "set lambda 2", 0.0, 1000.0);
    ddr_server_->register_variable("set_weight_diag", weight, [this](double val) { setWeightDiag(val); }, "set weight diag", 0.0, 1000.0);

    Eigen::Matrix3d Kp = getKp();
    ddr_server_->register_variable("kp_x", Kp(0,0), [this](double val) { setKpX(val); }, "Kp(0,0)", 0.0, 10000.0);
    ddr_server_->register_variable("kp_y", Kp(1,1), [this](double val) { setKpY(val); }, "Kp(1,1)", 0.0, 10000.0);
    ddr_server_->register_variable("kp_z", Kp(2,2), [this](double val) { setKpZ(val); }, "Kp(2,2)", 0.0, 10000.0);

    Eigen::Matrix3d Kd = getKd();
    ddr_server_->register_variable("kd_x", Kd(0,0), [this](double val) { setKdX(val); }, "Kd(0,0)", 0.0, 10000.0);
    ddr_server_->register_variable("kd_y", Kd(1,1), [this](double val) { setKdY(val); }, "Kd(1,1)", 0.0, 10000.0);
    ddr_server_->register_variable("kd_z", Kd(2,2), [this](double val) { setKdZ(val); }, "Kd(2,2)", 0.0, 10000.0);

    ddr_server_->publish_services_topics();*/
}

void ComImpl::loadParams()
{
    double lambda1, lambda2, weight;

    if (!task_nh_->get_parameter("gains." + _task_id + ".lambda1", lambda1))
    {
        RCLCPP_DEBUG(task_nh_->get_logger(), "No lambda1 gain given for task %s, using default value.", _task_id.c_str());
        lambda1 = getLambda();
    }
    if (!task_nh_->get_parameter("gains." + _task_id + ".lambda2", lambda2))
    {
        RCLCPP_DEBUG(task_nh_->get_logger(), "No lambda2 gain given for task %s, using default value.", _task_id.c_str());
        lambda2 = getLambda2();
    }
    if (!task_nh_->get_parameter("gains." + _task_id + ".weight", weight))
    {
        RCLCPP_DEBUG(task_nh_->get_logger(), "No weight gain given for task %s, using default value.", _task_id.c_str());
        weight = getWeight()(0,0);
    }

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
        if (!task_nh_->get_parameter("gains."+_task_id+".Kp." + wolf_controller_utils::_xyz[i] , Kp(i,i)))
        {
        RCLCPP_DEBUG(task_nh_->get_logger(), "No Kp.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wolf_controller_utils::_xyz[i].c_str(),_task_id.c_str(),robot_name_.c_str());
        use_identity = true;
        }
        if (!task_nh_->get_parameter("gains."+_task_id+".Kd."  + wolf_controller_utils::_xyz[i] , Kd(i,i)))
        {
        RCLCPP_DEBUG(task_nh_->get_logger(), "No Kd.%s gain given for task %s in the namespace: %s, using an identity matrix. ",wolf_controller_utils::_xyz[i].c_str(),_task_id.c_str(),robot_name_.c_str());
        use_identity = true;
        }
        // Check if the values are positive
        if(Kp(i,i)<0.0 || Kd(i,i)<0.0)
        throw std::runtime_error("Kp and Kd must be positive definite!");

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
