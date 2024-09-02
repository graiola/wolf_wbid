/**
 * @file cartesian.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the cartesian task wrapper for ROS2
 */

// WoLF
#include <wolf_wbid/task_ros2_wrappers/cartesian.h>
#include <wolf_controller_utils/converters.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace wolf_controller_utils;
using namespace wolf_wbid;

CartesianImpl::CartesianImpl(const std::string& robot_name,
                     const std::string& task_id,
                     const XBot::ModelInterface& robot,
                     const std::string& distal_link,
                     const std::string& base_link,
                     const OpenSoT::AffineHelper& qddot,
                     const double& period,
                     const bool& use_mesh)
  :Cartesian(robot_name,
             task_id,
             robot,
             distal_link,
             base_link,
             qddot,
             period,
             use_mesh)
  ,TaskRosHandler<wolf_msgs::msg::CartesianTask>(task_id,robot_name,period)
  ,is_continuous_(true)
  ,marker_name_(robot_name+"/wolf_controller/marker/"+_task_id)
  ,interactive_marker_server_(marker_name_,nh_)
  ,use_mesh_(use_mesh)
{
  // Get the URDF
  urdf_ = robot.getUrdf();

  // Get the urdf links (used for the base frame selection)
  urdf_.getLinks(links_);

  // Initialize buffers
  tmp_affine3d_.setIdentity();
  buffer_reference_pose_.initRT(tmp_affine3d_);
  tmp_vector6d_.setZero();
  buffer_reference_twist_.initRT(tmp_vector6d_);

  // Create the marker
  control_type_ = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
  makeMarker(getDistalLink(),getBaseLink(),static_cast<unsigned int>(control_type_),true);
  makeMenu();
  interactive_marker_server_.applyChanges();

  // Setup the interpolator
  trj_ = std::make_shared<CartesianTrajectory>(this);

  // Create the waypoints publisher
  waypoints_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseArray>(task_id + "/wp", 1);

  // Create the reference subscriber
  reference_sub_ = nh_->create_subscription<wolf_msgs::msg::Cartesian>(
      "reference/" + _task_id, 1000, std::bind(&CartesianImpl::referenceCallback, this, std::placeholders::_1));
}

void CartesianImpl::registerReconfigurableVariables()
{
  // Register dynamic reconfigurable variables (adapt ROS2 Dynamic Parameters or alternative service approach)
}

void CartesianImpl::loadParams()
{
  // Example of parameter handling with ROS2 parameters
  double lambda1, lambda2, weight;

  nh_->get_parameter_or("gains/" + _task_id + "/lambda1", lambda1, getLambda());
  nh_->get_parameter_or("gains/" + _task_id + "/lambda2", lambda2, getLambda2());
  nh_->get_parameter_or("gains/" + _task_id + "/weight", weight, getWeight()(0, 0));

  if (lambda1 < 0 || lambda2 < 0 || weight < 0)
    throw std::runtime_error("Lambda and weight must be positive!");

  buffer_lambda1_ = lambda1;
  buffer_lambda2_ = lambda2;
  buffer_weight_diag_ = weight;

  setLambda(lambda1, lambda2);
  setWeight(weight);

  Eigen::Matrix6d Kp = Eigen::Matrix6d::Zero();
  Eigen::Matrix6d Kd = Eigen::Matrix6d::Zero();

  bool use_identity = false;
  for (unsigned int i = 0; i < wolf_controller_utils::_cartesian_names.size(); i++)
  {
    std::string param_name_kp = "gains/" + _task_id + "/Kp/" + wolf_controller_utils::_cartesian_names[i];
    std::string param_name_kd = "gains/" + _task_id + "/Kd/" + wolf_controller_utils::_cartesian_names[i];

    nh_->get_parameter_or(param_name_kp, Kp(i, i), 1.0); // Default to 1.0 if not set
    nh_->get_parameter_or(param_name_kd, Kd(i, i), 1.0);

    if (Kp(i, i) < 0.0 || Kd(i, i) < 0.0)
      throw std::runtime_error("Kp and Kd must be positive definite!");
  }

  setKp(Kp);
  setKd(Kd);
}

void CartesianImpl::updateCost(const Eigen::VectorXd& x)
{
  cost_ = computeCost(x);
}

void CartesianImpl::publish()
{
  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = getBaseLink();
    rt_pub_->msg_.header.stamp = nh_->now();  // ROS2 now() function

    // ACTUAL VALUES
    getActualPose(tmp_affine3d_);
    getActualTwist(tmp_vector6d_);
    wolf_controller_utils::rotToRpy(tmp_affine3d_.linear(), tmp_vector3d_);
    wolf_controller_utils::affine3dToPose(tmp_affine3d_, rt_pub_->msg_.pose_actual);
    wolf_controller_utils::vector6dToTwist(tmp_vector6d_, rt_pub_->msg_.twist_actual);
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.rpy_actual);

    // REFERENCE VALUES
    getReference(tmp_affine3d_);
    tmp_vector6d_ = getCachedVelocityReference();
    wolf_controller_utils::rotToRpy(tmp_affine3d_.linear(), tmp_vector3d_);
    wolf_controller_utils::affine3dToPose(tmp_affine3d_, rt_pub_->msg_.pose_reference);
    wolf_controller_utils::vector6dToTwist(tmp_vector6d_, rt_pub_->msg_.twist_reference);
    wolf_controller_utils::vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.rpy_reference);

    // COST
    rt_pub_->msg_.cost = cost_;

    // PUBLISH
    rt_pub_->unlockAndPublish();
  }
}

bool CartesianImpl::reset()
{
  bool res = OpenSoT::tasks::acceleration::Cartesian::reset(); // Task's reset
  makeMarker(getDistalLink(), getBaseLink(), static_cast<unsigned int>(control_type_), true);
  menu_handler_.apply(interactive_marker_server_,marker_name_);
  interactive_marker_server_.applyChanges();
  waypoints_.clear();
  publishWP(waypoints_);
  trj_->reset();
  getActualPose(tmp_affine3d_);
  buffer_reference_pose_.initRT(tmp_affine3d_);
  tmp_vector6d_.setZero();
  buffer_reference_twist_.initRT(tmp_vector6d_);

  return res;
}

Eigen::Affine3d CartesianImpl::getPose(const std::string& base_link, const std::string& distal_link)
{
  static tf2_ros::Buffer buffer(nh_->get_clock());  // ROS2 tf2 buffer with clock
  static tf2_ros::TransformListener listener(buffer);

  geometry_msgs::msg::TransformStamped transformStamped;
  for (unsigned int i = 0; i < 10; ++i)
  {
    try
    {
      transformStamped = buffer.lookupTransform(base_link, distal_link, tf2::TimePointZero, tf2::durationFromSec(1.0));
      break; // Exit loop if transform is successfully obtained
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(nh_->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

  Eigen::Affine3d pose;
  pose = tf2::transformToEigen(transformStamped.transform);
  return pose;
}

void CartesianImpl::referenceCallback(const wolf_msgs::msg::Cartesian::SharedPtr msg)
{
  double period = period_;

  if (last_time_ != 0.0)
    period = (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) - last_time_;

  Eigen::Affine3d pose_reference = Eigen::Affine3d::Identity();
  Eigen::Vector6d twist_reference = Eigen::Vector6d::Zero();
  pose_reference = wolf_controller_utils::poseToAffine3d(msg->pose);
  twist_reference = wolf_controller_utils::twistToVector6d(msg->twist);

  if (!msg->header.frame_id.empty() && msg->header.frame_id != getBaseLink() && OPTIONS.set_ext_reference)
    setBaseLink(msg->header.frame_id);

  buffer_reference_pose_.writeFromNonRT(pose_reference);
  buffer_reference_twist_.writeFromNonRT(twist_reference);

  last_time_ = (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
}
