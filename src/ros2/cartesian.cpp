/**
 * @file cartesian.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the cartesian task wrapper for ROS2
 */

// WoLF
#include <wolf_wbid/ros2/cartesian.h>
#include <wolf_controller_utils/converters.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <wolf_controller_utils/ros2_param_getter.h>

using namespace wolf_controller_utils;
using namespace wolf_wbid;

CartesianImpl::CartesianImpl(const std::string& robot_name,
                     const std::string& task_id,
                     QuadrupedRobot& robot,
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
  ,marker_name_("wolf_controller/marker/"+_task_id)
  ,interactive_marker_server_(marker_name_,task_nh_)
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
  waypoints_pub_ = task_nh_->create_publisher<geometry_msgs::msg::PoseArray>(task_id + "/wp", 1);

  // Create the reference subscriber
  reference_sub_ = task_nh_->create_subscription<wolf_msgs::msg::Cartesian>(
      "reference/" + _task_id, 1000, std::bind(&CartesianImpl::referenceCallback, this, std::placeholders::_1));
}

void CartesianImpl::registerReconfigurableVariables()
{
  // Register dynamic reconfigurable variables (adapt ROS2 Dynamic Parameters or alternative service approach)
  double lambda1 = getLambda();
  double lambda2 = getLambda2();
  double weight  = getWeight()(0,0);
  Eigen::Matrix6d Kp = getKp();
  Eigen::Matrix6d Kd = getKd();

  // Load the tmp variables used in _update
  TaskWrapperInterface::setLambda1(lambda1);
  TaskWrapperInterface::setLambda2(lambda2);
  TaskWrapperInterface::setWeightDiag(weight);

  TaskWrapperInterface::setKpX(Kp(0,0));
  TaskWrapperInterface::setKpY(Kp(1,1));
  TaskWrapperInterface::setKpZ(Kp(2,2));
  TaskWrapperInterface::setKpRoll(Kp(3,3));
  TaskWrapperInterface::setKpPitch(Kp(4,4));
  TaskWrapperInterface::setKpYaw(Kp(5,5));

  TaskWrapperInterface::setKdX(Kd(0,0));
  TaskWrapperInterface::setKdY(Kd(1,1));
  TaskWrapperInterface::setKdZ(Kd(2,2));
  TaskWrapperInterface::setKdRoll(Kd(3,3));
  TaskWrapperInterface::setKdPitch(Kd(4,4));
  TaskWrapperInterface::setKdYaw(Kd(5,5));
}

void CartesianImpl::loadParams()
{
  // Example of parameter handling with ROS2 parameters
  double lambda1, lambda2, weight;
  lambda1 = getLambda();
  lambda2 = getLambda2();
  weight  = getWeight()(0, 0);

  lambda1 = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".lambda1", lambda1);
  lambda2 = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".lambda2", lambda2);
  weight  = get_double_parameter_from_remote_node("wolf_controller/gains."+_task_id+".weight",  weight);

  if (lambda1 < 0.0 || lambda2 < 0.0 || weight < 0.0)
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
    Kp(i, i) = get_double_parameter_from_remote_node("wolf_controller/gains." + _task_id + ".Kp." + wolf_controller_utils::_cartesian_names[i], 0.0);
    Kd(i, i) = get_double_parameter_from_remote_node("wolf_controller/gains." + _task_id + ".Kd." + wolf_controller_utils::_cartesian_names[i], 0.0);

    if (Kp(i, i) < 0.0 || Kd(i, i) < 0.0)
      use_identity = true;
  }

  if(use_identity)
  {
    Kp = Eigen::Matrix6d::Identity();
    Kd = Eigen::Matrix6d::Identity();
  }

  setKp(Kp);
  setKd(Kd);
}

void CartesianImpl::_update(const Eigen::VectorXd& x)
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_,buffer_lambda2_);
  if(OPTIONS.set_ext_weight)
    setWeight(buffer_weight_diag_);
  if(OPTIONS.set_ext_gains)
  {
    tmp_matrix6d_.setZero();
    tmp_matrix6d_(0,0) = buffer_kp_x_;
    tmp_matrix6d_(1,1) = buffer_kp_y_;
    tmp_matrix6d_(2,2) = buffer_kp_z_;
    tmp_matrix6d_(3,3) = buffer_kp_roll_;
    tmp_matrix6d_(4,4) = buffer_kp_pitch_;
    tmp_matrix6d_(5,5) = buffer_kp_yaw_;
    setKp(tmp_matrix6d_);
    tmp_matrix6d_.setZero();
    tmp_matrix6d_(0,0) = buffer_kd_x_;
    tmp_matrix6d_(1,1) = buffer_kd_y_;
    tmp_matrix6d_(2,2) = buffer_kd_z_;
    tmp_matrix6d_(3,3) = buffer_kd_roll_;
    tmp_matrix6d_(4,4) = buffer_kd_pitch_;
    tmp_matrix6d_(5,5) = buffer_kd_yaw_;
    setKd(tmp_matrix6d_);
  }
  if(OPTIONS.set_ext_reference)
  {
    if(is_continuous_) // Direct control
    {
      //setReference(*buffer_reference_pose_.readFromRT());
      setReference(*buffer_reference_pose_.readFromRT(),*buffer_reference_twist_.readFromRT());
    }
    else // Interpolation
    {
      trj_->update(period_);
      trj_->getReference(tmp_affine3d_,&tmp_vector6d_);
      setReference(tmp_affine3d_,tmp_vector6d_);
    }
  }
  OpenSoT::tasks::acceleration::Cartesian::_update(x);
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
    rt_pub_->msg_.header.stamp = task_nh_->now();  // ROS2 now() function

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

void CartesianImpl::makeMarker(const std::string &distal_link, const std::string &base_link, unsigned int interaction_mode, bool show)
{
  RCLCPP_DEBUG(rclcpp::get_logger("cartesian_impl"), "Creating marker %s -> %s", base_link.c_str(), distal_link.c_str());

  interactive_marker_.header.frame_id = base_link;
  interactive_marker_.scale = 0.5;

  interactive_marker_.name = distal_link;
  interactive_marker_.description = "";

  // Insert STL
  makeSTLControl(interactive_marker_);
  interactive_marker_.controls[0].interaction_mode = interaction_mode;

  if (show)
  {
    createInteractiveMarkerControl(1, 1, 0, 0, interaction_mode);
    createInteractiveMarkerControl(1, 0, 1, 0, interaction_mode);
    createInteractiveMarkerControl(1, 0, 0, 1, interaction_mode);
  }

  Eigen::Affine3d start_pose;
  getActualPose(start_pose);
  affine3dToVisualizationPose(start_pose, interactive_marker_);

  // Use std::bind instead of boost::bind
  interactive_marker_server_.insert(interactive_marker_, std::bind(&CartesianImpl::processFeedback, this, std::placeholders::_1));
}

void CartesianImpl::createInteractiveMarkerControl(const double qw, const double qx, const double qy, const double qz,
                                                   const unsigned int interaction_mode)
{
  visualization_msgs::msg::InteractiveMarkerControl tmp_control;
  tmp_control.orientation.w = qw;
  tmp_control.orientation.x = qx;
  tmp_control.orientation.y = qy;
  tmp_control.orientation.z = qz;

  if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D)
  {
    tmp_control.name = "rotate_x";
    tmp_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
    tmp_control.name = "move_x";
    tmp_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
  }
  else if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D)
  {
    tmp_control.name = "move_x";
    tmp_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
  }
  else if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D)
  {
    tmp_control.name = "rotate_x";
    tmp_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
  }
  else
  {
    throw std::invalid_argument("Invalid interaction mode!");
  }
}

visualization_msgs::msg::InteractiveMarkerControl& CartesianImpl::makeSTLControl(visualization_msgs::msg::InteractiveMarker& msg)
{
  visualization_msgs::msg::InteractiveMarkerControl tmp_control;
  tmp_control.always_visible = true;

  if (use_mesh_ && urdf_.getLink(getDistalLink()) != nullptr)
  {
    tmp_control.markers.push_back(makeSTL(msg));
  }
  else
  {
    tmp_control.markers.push_back(makeSphere(msg));
  }

  msg.controls.push_back(tmp_control);

  return msg.controls.back();
}

visualization_msgs::msg::Marker CartesianImpl::makeSphere(visualization_msgs::msg::InteractiveMarker& msg)
{
  marker_.type = visualization_msgs::msg::Marker::SPHERE;
  marker_.scale.x = msg.scale * 0.45;
  marker_.scale.y = msg.scale * 0.45;
  marker_.scale.z = msg.scale * 0.45;
  marker_.color.r = 0.5;
  marker_.color.g = 0.5;
  marker_.color.b = 1.5;
  marker_.color.a = 1.0;
  return marker_;
}

visualization_msgs::msg::Marker CartesianImpl::makeSTL(visualization_msgs::msg::InteractiveMarker& msg)
{
  auto distal_link = getDistalLink();
  auto link = urdf_.getLink(distal_link);
  auto controlled_link = link;

  while (!link->visual)
  {
    if (!link->parent_joint)
    {
      RCLCPP_WARN_STREAM(task_nh_->get_logger(), "Unable to find mesh for link " << distal_link);
      return makeSphere(msg);
    }
    link = urdf_.getLink(link->parent_joint->parent_link_name);
  }

  Eigen::Affine3d actual_pose = getPose(controlled_link->name, link->name);
  affine3dToVisualizationPose(actual_pose, marker_);

  marker_.color.r = 0.5;
  marker_.color.g = 0.5;
  marker_.color.b = 0.5;

  if (link->visual->geometry->type == urdf::Geometry::MESH)
  {
    marker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

    auto mesh = std::static_pointer_cast<urdf::Mesh>(link->visual->geometry);
    marker_.mesh_resource = mesh->filename;
    marker_.scale.x = mesh->scale.x;
    marker_.scale.y = mesh->scale.y;
    marker_.scale.z = mesh->scale.z;
  }
  else if (link->visual->geometry->type == urdf::Geometry::BOX)
  {
    marker_.type = visualization_msgs::msg::Marker::CUBE;

    auto box = std::static_pointer_cast<urdf::Box>(link->visual->geometry);
    marker_.scale.x = box->dim.x;
    marker_.scale.y = box->dim.y;
    marker_.scale.z = box->dim.z;
  }
  else if (link->visual->geometry->type == urdf::Geometry::CYLINDER)
  {
    marker_.type = visualization_msgs::msg::Marker::CYLINDER;

    auto cylinder = std::static_pointer_cast<urdf::Cylinder>(link->visual->geometry);
    marker_.scale.x = marker_.scale.y = cylinder->radius;
    marker_.scale.z = cylinder->length;
  }
  else if (link->visual->geometry->type == urdf::Geometry::SPHERE)
  {
    marker_.type = visualization_msgs::msg::Marker::SPHERE;

    auto sphere = std::static_pointer_cast<urdf::Sphere>(link->visual->geometry);
    marker_.scale.x = marker_.scale.y = marker_.scale.z = 2. * sphere->radius;
  }

  marker_.color.a = 0.9;
  return marker_;
}

void CartesianImpl::makeMenu()
{
  continuous_control_entry_ = menu_handler_.insert("Marker Ctrl", std::bind(&CartesianImpl::setContinuousCtrl, this, std::placeholders::_1));
  menu_handler_.setCheckState(continuous_control_entry_, interactive_markers::MenuHandler::CHECKED);

  way_point_entry_ = menu_handler_.insert("Add WayPoint");
  menu_handler_.setVisible(way_point_entry_, false);

  T_entry_ = menu_handler_.insert(way_point_entry_, "T [sec]");
  for (unsigned int i = 0; i < 10; i++)
  {
    std::ostringstream sec_opt;
    double sec = (i + 1) * 0.5;
    sec_opt << sec;
    T_last_ = menu_handler_.insert(T_entry_, sec_opt.str(), std::bind(&CartesianImpl::wayPointCallBack, this, std::placeholders::_1, sec));
    menu_handler_.setCheckState(T_last_, interactive_markers::MenuHandler::UNCHECKED);
  }

  reset_all_way_points_entry_ = menu_handler_.insert(way_point_entry_, "Reset All", std::bind(&CartesianImpl::resetAllWayPoints, this, std::placeholders::_1));
  reset_lastway_point_entry_ = menu_handler_.insert(way_point_entry_, "Reset Last", std::bind(&CartesianImpl::resetLastWayPoints, this, std::placeholders::_1));
  send_way_points_entry_ = menu_handler_.insert(way_point_entry_, "Send", std::bind(&CartesianImpl::sendWayPoints, this, std::placeholders::_1));

  base_link_entry_ = menu_handler_.insert("Base Link");
  menu_handler_.setVisible(base_link_entry_, true);

  std::string distal_link = getDistalLink();
  std::string base_link = getBaseLink();
  for (unsigned int i = 0; i < links_.size(); ++i)
  {
    if (distal_link != links_.at(i)->name)
    {
      interactive_markers::MenuHandler::EntryHandle link_entry =
          menu_handler_.insert(base_link_entry_, links_.at(i)->name, std::bind(&CartesianImpl::changeBaseLink, this, std::placeholders::_1, links_.at(i)->name));

      if (base_link.compare("world") == 0 && (links_.at(i)->name).compare("world") == 0)
      {
        menu_handler_.setCheckState(link_entry, interactive_markers::MenuHandler::CHECKED);
      }
      else if (base_link.compare(links_.at(i)->name) == 0)
      {
        menu_handler_.setCheckState(link_entry, interactive_markers::MenuHandler::CHECKED);
      }
      else
        menu_handler_.setCheckState(link_entry, interactive_markers::MenuHandler::UNCHECKED);

      map_link_entries_[links_.at(i)->name] = link_entry;
    }
  }

  menu_control_.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  menu_control_.always_visible = true;

  interactive_marker_.controls.push_back(menu_control_);
  menu_handler_.apply(interactive_marker_server_, interactive_marker_.name);
}


void CartesianImpl::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("cartesian_impl"), feedback->marker_name << " is now at "
                        << feedback->pose.position.x << ", " << feedback->pose.position.y
                        << ", " << feedback->pose.position.z << " frame " << feedback->header.frame_id);

  Eigen::Affine3d pose_reference = Eigen::Affine3d::Identity();
  Eigen::Vector6d twist_reference = Eigen::Vector6d::Zero();
  pose_reference = wolf_controller_utils::poseToAffine3d(feedback->pose); // Replace tf::poseMsgToEigen with custom util

  if (is_continuous_)
  {
    buffer_reference_pose_.writeFromNonRT(pose_reference);
    buffer_reference_twist_.writeFromNonRT(twist_reference);
  }
}


Eigen::Affine3d CartesianImpl::getPose(const std::string& base_link, const std::string& distal_link)
{
  static tf2_ros::Buffer buffer(task_nh_->get_clock());  // ROS2 tf2 buffer with clock
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
      RCLCPP_ERROR(task_nh_->get_logger(), "%s", ex.what());
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

void CartesianImpl::sendWayPoints(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
{
  if(waypoints_.empty()) return;

  // Accumulate durations for waypoints
  std::partial_sum(T_.begin(), T_.end(), T_.begin());

  CartesianTrajectory::WayPointVector wpv;

  for(size_t i = 0; i < waypoints_.size(); i++)
  {
    CartesianTrajectory::WayPoint wp;
    wp.T_ref = wolf_controller_utils::poseToAffine3d(waypoints_[i]); // Convert pose to Eigen::Affine3d
    wp.duration = T_.at(i); // Set duration

    wpv.push_back(wp); // Add waypoint to vector
  }

  trj_->setWayPoints(wpv); // Send waypoints to trajectory handler
  publishWP(waypoints_); // Publish waypoints

  // Clear waypoints and durations
  waypoints_.clear();
  T_.clear();
}

void CartesianImpl::resetMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
{
  clearMarker(); // Clear current marker
  spawnMarker(); // Respawn marker
}

void CartesianImpl::resetLastWayPoints(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CartesianImpl"), "Reset last waypoint!");

  if (!T_.empty()) T_.pop_back();
  if (!waypoints_.empty()) waypoints_.pop_back();

  clearMarker();

  if (waypoints_.empty()) {
    spawnMarker(); // Spawn if no waypoints left
  } else {
    // Handle remaining waypoints
    if (interactive_marker_server_.empty()) {
      poseToVisualizationPose(waypoints_.back(), interactive_marker_); // Convert to visualization pose
      interactive_marker_server_.insert(interactive_marker_, std::bind(&CartesianImpl::processFeedback, this, std::placeholders::_1)); // Use std::bind
      menu_handler_.reApply(interactive_marker_server_);
      interactive_marker_server_.applyChanges(); // Apply changes
    }
  }

  publishWP(waypoints_); // Republish waypoints
}

void CartesianImpl::resetAllWayPoints(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  T_.clear(); // Clear all durations
  waypoints_.clear(); // Clear all waypoints
  resetMarker(feedback); // Reset marker
  publishWP(waypoints_); // Publish empty waypoint list
  RCLCPP_INFO(rclcpp::get_logger("CartesianImpl"), "Reset all waypoints!");
}

void CartesianImpl::wayPointCallBack(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback, double T)
{
  double x, y, z, qx, qy, qz, qw;
  x  = feedback->pose.position.x;
  y  = feedback->pose.position.y;
  z  = feedback->pose.position.z;
  qx = feedback->pose.orientation.x;
  qy = feedback->pose.orientation.y;
  qz = feedback->pose.orientation.z;
  qw = feedback->pose.orientation.w;

  if (!is_continuous_) {
    RCLCPP_INFO(rclcpp::get_logger("CartesianImpl"),
                "\n %s set waypoint wrt %s @: \n pos = [%f, %f, %f],\n orient = [%f, %f, %f, %f],\n of %.1f secs",
                interactive_marker_.name.c_str(),
                getBaseLink().c_str(),
                x, y, z, qx, qy, qz, qw, T);

    waypoints_.push_back(feedback->pose); // Add waypoint
    T_.push_back(T); // Add duration
    publishWP(waypoints_); // Publish waypoints
  }
}

void CartesianImpl::publishWP(const std::vector<geometry_msgs::msg::Pose>& wps)
{
  geometry_msgs::msg::PoseArray msg; // Create PoseArray message

  for (const auto& wp : wps) {
    msg.poses.push_back(wp); // Add waypoints to the message
  }

  msg.header.frame_id = getBaseLink(); // Set frame ID
  msg.header.stamp = rclcpp::Clock().now(); // Use rclcpp::Clock for time stamp

  waypoints_pub_->publish(msg); // Publish the waypoints
}

bool CartesianImpl::clearMarker()
{
  if (!interactive_marker_server_.empty()) {
    interactive_marker_server_.erase(interactive_marker_.name); // Erase marker from server
    interactive_marker_server_.applyChanges(); // Apply changes to server
  }
  return true;
}

bool CartesianImpl::spawnMarker()
{
  if (interactive_marker_server_.empty()) {
    Eigen::Affine3d start_pose;
    getActualPose(start_pose); // Get current pose
    affine3dToVisualizationPose(start_pose, interactive_marker_); // Convert pose for visualization
    interactive_marker_server_.insert(interactive_marker_, std::bind(&CartesianImpl::processFeedback, this, std::placeholders::_1)); // Insert marker with std::bind
    menu_handler_.reApply(interactive_marker_server_); // Reapply menu handler
    interactive_marker_server_.applyChanges(); // Apply changes to the server
  }
  return true;
}

void CartesianImpl::setContinuousCtrl(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
{
  is_continuous_ = !is_continuous_; // Toggle continuous control

  if (!is_continuous_) {
    menu_handler_.setCheckState(continuous_control_entry_, interactive_markers::MenuHandler::UNCHECKED);
    menu_handler_.setVisible(way_point_entry_, true); // Make waypoints visible
    waypoints_.clear(); // Clear waypoints
    T_.clear(); // Clear durations
    trj_->reset(); // Reset trajectory
  } else {
    menu_handler_.setCheckState(continuous_control_entry_, interactive_markers::MenuHandler::CHECKED);
    menu_handler_.setVisible(way_point_entry_, false); // Hide waypoints
    waypoints_.clear();
    T_.clear();

    // Initialize pose and twist for continuous control
    Eigen::Affine3d pose;
    Eigen::Vector6d twist = Eigen::Vector6d::Zero();
    getActualPose(pose);
    buffer_reference_pose_.writeFromNonRT(pose);
    buffer_reference_twist_.writeFromNonRT(twist);
  }

  clearMarker(); // Clear marker
  spawnMarker(); // Spawn new marker
  publishWP(waypoints_); // Publish waypoints

  menu_handler_.reApply(interactive_marker_server_); // Reapply menu handler
  interactive_marker_server_.applyChanges(); // Apply changes to the server
}

void CartesianImpl::changeBaseLink(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback, std::string new_base_link)
{
  int entry_id = feedback->menu_entry_id;

  // Get the title for the entry from the menu handler
  menu_handler_.getTitle(entry_id, new_base_link);

  // If the new base link is the same as the current one, do nothing
  if (new_base_link.compare(getBaseLink()) == 0)
    return;

  // If successfully set the new base link
  if (setBaseLink(new_base_link))
  {
    // Update the internal state and reset necessary parameters
    update(Eigen::VectorXd(1)); // Dummy update call, can be changed based on specific requirements
    reset();

    // Update the menu entries' check state based on the new base link
    for (auto& tmp_map : map_link_entries_)
    {
      if (tmp_map.first == new_base_link)
        menu_handler_.setCheckState(tmp_map.second, interactive_markers::MenuHandler::CHECKED);
      else
        menu_handler_.setCheckState(tmp_map.second, interactive_markers::MenuHandler::UNCHECKED);
    }

    // Apply the changes to the menu handler and the interactive marker server
    menu_handler_.reApply(interactive_marker_server_);
    interactive_marker_server_.applyChanges();
  }
}
