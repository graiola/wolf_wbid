/**
 * @file cartesian.cpp
 * @author Gennaro Raiola
 * @brief Cartesian task wrapper for ROS (OpenSoT-free)
 */

#include <wolf_wbid/task_ros_wrappers/cartesian.h>

#include <wolf_wbid/quadruped_robot.h>
#include <wolf_wbid/wbid/id_variables.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>

// (optional) if you used WORLD_FRAME_NAME in old code
#include <wolf_wbid/task_interface.h>

using namespace wolf_controller_utils;
using namespace wolf_wbid;

CartesianImpl::CartesianImpl(const std::string& robot_name,
                             const std::string& task_id,
                             QuadrupedRobot& robot,
                             const std::string& distal_link,
                             const std::string& base_link,
                             const IDVariables& vars,
                             const double& period,
                             const bool& use_mesh)
: Cartesian(robot_name, task_id, robot, distal_link, base_link, vars, period, use_mesh)
, TaskRosHandler<wolf_msgs::CartesianTask>(task_id, robot_name, period)
, robot_(robot)
, interactive_marker_server_(robot_name + "/wolf_controller/marker/" + task_id)
, use_mesh_(use_mesh)
{
  // Get the urdf (used for the mesh)
  urdf_ = robot_.getUrdf();

  // Get the urdf links (used for the base frame selection)
  urdf_.getLinks(links_);

  // Initialize buffers
  tmp_affine3d_.setIdentity();
  buffer_reference_pose_.initRT(tmp_affine3d_);
  tmp_vector6d_.setZero();
  buffer_reference_twist_.initRT(tmp_vector6d_);

  // Create the marker
  control_type_ = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  makeMarker(getDistalLink(), getBaseLink(), static_cast<unsigned int>(control_type_), true);
  makeMenu();
  interactive_marker_server_.applyChanges();

  // Setup the interpolator
  trj_ = std::make_shared<CartesianTrajectory>(this);

  // Create the waypoints publisher
  waypoints_pub_ = nh_.advertise<geometry_msgs::PoseArray>(task_id + "/wp", 1, true);

  // Create the reference subscriber
  reference_sub_ = nh_.subscribe("reference/" + task_id, 1000, &CartesianImpl::referenceCallback, this);
}

void CartesianImpl::registerReconfigurableVariables()
{
  const double lambda1 = getLambda();
  const double lambda2 = getLambda2();
  const double weight  = getWeight()(0,0);

  ddr_server_->registerVariable<double>("set_lambda_1", lambda1,
      boost::bind(&TaskWrapperInterface::setLambda1, this, _1),
      "set lambda 1", 0.0, 1000.0);

  ddr_server_->registerVariable<double>("set_lambda_2", lambda2,
      boost::bind(&TaskWrapperInterface::setLambda2, this, _1),
      "set lambda 2", 0.0, 1000.0);

  ddr_server_->registerVariable<double>("set_weight_diag", weight,
      boost::bind(&TaskWrapperInterface::setWeightDiag, this, _1),
      "set weight diag", 0.0, 1000.0);

  const Eigen::Matrix<double,6,6> Kp = getKp();
  ddr_server_->registerVariable<double>("kp_x",     Kp(0,0), boost::bind(&TaskWrapperInterface::setKpX,this,_1),     "Kp(0,0)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_y",     Kp(1,1), boost::bind(&TaskWrapperInterface::setKpY,this,_1),     "Kp(1,1)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_z",     Kp(2,2), boost::bind(&TaskWrapperInterface::setKpZ,this,_1),     "Kp(2,2)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_roll",  Kp(3,3), boost::bind(&TaskWrapperInterface::setKpRoll,this,_1),  "Kp(3,3)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_pitch", Kp(4,4), boost::bind(&TaskWrapperInterface::setKpPitch,this,_1), "Kp(4,4)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kp_yaw",   Kp(5,5), boost::bind(&TaskWrapperInterface::setKpYaw,this,_1),   "Kp(5,5)", 0.0, 10000.0);

  const Eigen::Matrix<double,6,6> Kd = getKd();
  ddr_server_->registerVariable<double>("kd_x",     Kd(0,0), boost::bind(&TaskWrapperInterface::setKdX,this,_1),     "Kd(0,0)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_y",     Kd(1,1), boost::bind(&TaskWrapperInterface::setKdY,this,_1),     "Kd(1,1)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_z",     Kd(2,2), boost::bind(&TaskWrapperInterface::setKdZ,this,_1),     "Kd(2,2)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_roll",  Kd(3,3), boost::bind(&TaskWrapperInterface::setKdRoll,this,_1),  "Kd(3,3)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_pitch", Kd(4,4), boost::bind(&TaskWrapperInterface::setKdPitch,this,_1), "Kd(4,4)", 0.0, 10000.0);
  ddr_server_->registerVariable<double>("kd_yaw",   Kd(5,5), boost::bind(&TaskWrapperInterface::setKdYaw,this,_1),   "Kd(5,5)", 0.0, 10000.0);

  ddr_server_->publishServicesTopics();
}

void CartesianImpl::loadParams()
{
  double lambda1, lambda2, weight;

  if (!nh_.getParam("gains/"+task_name_+"/lambda1", lambda1))
    lambda1 = getLambda();

  if (!nh_.getParam("gains/"+task_name_+"/lambda2", lambda2))
    lambda2 = getLambda2();

  if (!nh_.getParam("gains/"+task_name_+"/weight", weight))
    weight = getWeight()(0,0);

  if(lambda1 < 0 || lambda2 < 0 || weight < 0)
    throw std::runtime_error("Lambda and weight must be positive!");

  buffer_lambda1_ = lambda1;
  buffer_lambda2_ = lambda2;
  buffer_weight_diag_ = weight;

  // push immediately (startup)
  setLambda(lambda1, lambda2);

  Eigen::Matrix<double,6,6> W = Eigen::Matrix<double,6,6>::Identity();
  W.diagonal().setConstant(weight);
  setWeight(W);

  Eigen::Matrix<double,6,6> Kp = Eigen::Matrix<double,6,6>::Zero();
  Eigen::Matrix<double,6,6> Kd = Eigen::Matrix<double,6,6>::Zero();

  bool use_identity = false;
  static const std::vector<std::string> cart_names = {"x","y","z","roll","pitch","yaw"};

  for(int i=0;i<6;i++)
  {
    if (!nh_.getParam("gains/"+task_name_+"/Kp/"+cart_names[i], Kp(i,i)))
      use_identity = true;

    if (!nh_.getParam("gains/"+task_name_+"/Kd/"+cart_names[i], Kd(i,i)))
      use_identity = true;

    if(Kp(i,i) < 0.0 || Kd(i,i) < 0.0)
      throw std::runtime_error("Kp and Kd must be positive!");
  }

  if(use_identity)
  {
    Kp.setIdentity();
    Kd.setIdentity();
  }

  buffer_kp_x_     = Kp(0,0);
  buffer_kp_y_     = Kp(1,1);
  buffer_kp_z_     = Kp(2,2);
  buffer_kp_roll_  = Kp(3,3);
  buffer_kp_pitch_ = Kp(4,4);
  buffer_kp_yaw_   = Kp(5,5);

  buffer_kd_x_     = Kd(0,0);
  buffer_kd_y_     = Kd(1,1);
  buffer_kd_z_     = Kd(2,2);
  buffer_kd_roll_  = Kd(3,3);
  buffer_kd_pitch_ = Kd(4,4);
  buffer_kd_yaw_   = Kd(5,5);

  setKp(Kp);
  setKd(Kd);

  std::string type;
  if(nh_.getParam("gains/"+task_name_+"/type", type))
  {
    if(type == "acceleration")
      setGainType(CartesianTask::GainType::Acceleration);
    else if(type == "force")
      setGainType(CartesianTask::GainType::Force);
    else
      throw std::runtime_error("Wrong gain type, possible values are 'acceleration' or 'force'");
  }
}

void CartesianImpl::applyExternalKnobs()
{
  if(OPTIONS.set_ext_lambda)
    setLambda(buffer_lambda1_.load(), buffer_lambda2_.load());

  if(OPTIONS.set_ext_weight)
  {
    Eigen::Matrix<double,6,6> W = Eigen::Matrix<double,6,6>::Identity();
    W.diagonal().setConstant(buffer_weight_diag_.load());
    setWeight(W);
  }

  if(OPTIONS.set_ext_gains)
  {
    Eigen::Matrix<double,6,6> Kp = Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,6> Kd = Eigen::Matrix<double,6,6>::Zero();

    Kp(0,0)=buffer_kp_x_.load();     Kp(1,1)=buffer_kp_y_.load();     Kp(2,2)=buffer_kp_z_.load();
    Kp(3,3)=buffer_kp_roll_.load();  Kp(4,4)=buffer_kp_pitch_.load(); Kp(5,5)=buffer_kp_yaw_.load();

    Kd(0,0)=buffer_kd_x_.load();     Kd(1,1)=buffer_kd_y_.load();     Kd(2,2)=buffer_kd_z_.load();
    Kd(3,3)=buffer_kd_roll_.load();  Kd(4,4)=buffer_kd_pitch_.load(); Kd(5,5)=buffer_kd_yaw_.load();

    setKp(Kp);
    setKd(Kd);
  }
}

void CartesianImpl::applyExternalReference()
{
  if(!OPTIONS.set_ext_reference)
    return;

  if(is_continuous_)
  {
    setReference(*buffer_reference_pose_.readFromRT(),
                 *buffer_reference_twist_.readFromRT());
  }
  else
  {
    trj_->update(period_);
    trj_->getReference(tmp_affine3d_, &tmp_vector6d_);
    setReference(tmp_affine3d_, tmp_vector6d_);
  }
}

void CartesianImpl::updateCost(const Eigen::VectorXd& /*x*/)
{
  // Proxy cost for debug
  cost_ = getError().squaredNorm() + getVelocityError().squaredNorm();
}

void CartesianImpl::publish()
{
  if(rt_pub_->trylock())
  {
    rt_pub_->msg_.header.frame_id = getBaseLink();
    rt_pub_->msg_.header.stamp = ros::Time::now();

    // ACTUAL VALUES
    Eigen::Affine3d T_act;
    Eigen::Matrix<double,6,1> V_act;
    getActualPose(T_act);
    getActualTwist(V_act);

    rotToRpy(T_act.linear(), tmp_vector3d_);
    affine3dToPose(T_act, rt_pub_->msg_.pose_actual);
    vector6dToTwist(V_act, rt_pub_->msg_.twist_actual);
    vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.rpy_actual);

    // REFERENCE VALUES
    Eigen::Affine3d T_ref;
    getReference(T_ref);
    tmp_vector6d_ = getCachedVelocityReference();

    rotToRpy(T_ref.linear(), tmp_vector3d_);
    affine3dToPose(T_ref, rt_pub_->msg_.pose_reference);
    vector6dToTwist(tmp_vector6d_, rt_pub_->msg_.twist_reference);
    vector3dToVector3(tmp_vector3d_, rt_pub_->msg_.rpy_reference);

    // COST
    rt_pub_->msg_.cost = cost_;

    rt_pub_->unlockAndPublish();
  }
}

bool CartesianImpl::reset()
{
  // reset task core
  CartesianTask::reset();

  // reset marker UI
  makeMarker(getDistalLink(), getBaseLink(),
             static_cast<unsigned int>(control_type_), true);
  menu_handler_.reApply(interactive_marker_server_);
  interactive_marker_server_.applyChanges();

  waypoints_.clear();
  publishWP(waypoints_);
  trj_->reset();

  // init reference buffers from current pose
  Eigen::Affine3d pose;
  getActualPose(pose);
  buffer_reference_pose_.writeFromNonRT(pose);

  Eigen::Matrix<double,6,1> twist = Eigen::Matrix<double,6,1>::Zero();
  buffer_reference_twist_.writeFromNonRT(twist);

  return true;
}

void CartesianImpl::makeMarker(const std::string &distal_link, const std::string &base_link,
                               unsigned int interaction_mode, bool show)
{
  ROS_DEBUG("Creating marker %s -> %s\n", base_link.c_str(), distal_link.c_str());

  interactive_marker_.header.frame_id = base_link;
  interactive_marker_.scale = 0.5;

  interactive_marker_.name = distal_link;
  interactive_marker_.description = "";

  // Insert STL
  makeSTLControl(interactive_marker_);
  interactive_marker_.controls[0].interaction_mode = interaction_mode;

  if(show)
  {
    createInteractiveMarkerControl(1,1,0,0,interaction_mode);
    createInteractiveMarkerControl(1,0,1,0,interaction_mode);
    createInteractiveMarkerControl(1,0,0,1,interaction_mode);
  }

  Eigen::Affine3d start_pose;
  getActualPose(start_pose);
  affine3dToVisualizationPose(start_pose, interactive_marker_);

  interactive_marker_server_.insert(interactive_marker_,
                                    boost::bind(&CartesianImpl::processFeedback, this, _1));
}

void CartesianImpl::createInteractiveMarkerControl(const double qw, const double qx, const double qy, const double qz,
                                                   const unsigned int interaction_mode)
{
  visualization_msgs::InteractiveMarkerControl tmp_control;
  tmp_control.orientation.w = qw;
  tmp_control.orientation.x = qx;
  tmp_control.orientation.y = qy;
  tmp_control.orientation.z = qz;

  if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D)
  {
    tmp_control.name = "rotate_x";
    tmp_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
    tmp_control.name = "move_x";
    tmp_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
  }
  else if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D)
  {
    tmp_control.name = "move_x";
    tmp_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
  }
  else if(interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D)
  {
    tmp_control.name = "rotate_x";
    tmp_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker_.controls.push_back(tmp_control);
  }
  else throw std::invalid_argument("Invalid interaction mode!");
}

void CartesianImpl::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_DEBUG_STREAM( feedback->marker_name << " is now at "
                    << feedback->pose.position.x << ", " << feedback->pose.position.y
                    << ", " << feedback->pose.position.z << " frame " << feedback->header.frame_id );

  Eigen::Affine3d pose_reference = Eigen::Affine3d::Identity();
  Eigen::Matrix<double,6,1> twist_reference = Eigen::Matrix<double,6,1>::Zero();

  pose_reference = wolf_controller_utils::poseToAffine3d(feedback->pose);

  if(is_continuous_)
  {
    buffer_reference_pose_.writeFromNonRT(pose_reference);
    buffer_reference_twist_.writeFromNonRT(twist_reference);
  }
}

void CartesianImpl::referenceCallback(const wolf_msgs::Cartesian::ConstPtr& msg)
{
  Eigen::Affine3d pose_reference = Eigen::Affine3d::Identity();
  Eigen::Matrix<double,6,1> twist_reference = Eigen::Matrix<double,6,1>::Zero();

  pose_reference  = wolf_controller_utils::poseToAffine3d(msg->pose);
  twist_reference = wolf_controller_utils::twistToVector6d(msg->twist);

  // Check if reference frame changed
  if(msg->header.frame_id != "" && msg->header.frame_id != getBaseLink() && OPTIONS.set_ext_reference)
    setBaseLink(msg->header.frame_id);

  buffer_reference_pose_.writeFromNonRT(pose_reference);
  buffer_reference_twist_.writeFromNonRT(twist_reference);

  last_time_ = msg->header.stamp.toSec();
}

visualization_msgs::InteractiveMarkerControl& CartesianImpl::makeSTLControl(visualization_msgs::InteractiveMarker& msg)
{
  visualization_msgs::InteractiveMarkerControl tmp_control;
  tmp_control.always_visible = true;

  if(use_mesh_ && urdf_.getLink(getDistalLink()) != nullptr)
    tmp_control.markers.push_back(makeSTL(msg));
  else
    tmp_control.markers.push_back(makeSphere(msg));

  msg.controls.push_back(tmp_control);
  return msg.controls.back();
}

visualization_msgs::Marker CartesianImpl::makeSphere(visualization_msgs::InteractiveMarker& msg)
{
  marker_.type = visualization_msgs::Marker::SPHERE;
  marker_.scale.x = msg.scale * 0.45;
  marker_.scale.y = msg.scale * 0.45;
  marker_.scale.z = msg.scale * 0.45;
  marker_.color.r = 0.5;
  marker_.color.g = 0.5;
  marker_.color.b = 1.5;
  marker_.color.a = 1.0;
  return marker_;
}

Eigen::Affine3d CartesianImpl::getPoseTF(const std::string& base_link, const std::string& distal_link)
{
  static tf2_ros::Buffer buffer;
  static tf2_ros::TransformListener listener(buffer);

  geometry_msgs::TransformStamped transformStamped;
  for(unsigned int i = 0; i < 10; ++i)
  {
    try
    {
      transformStamped = buffer.lookupTransform(base_link, distal_link, ros::Time(0), ros::Duration(1.0));
      break;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  return tf2::transformToEigen(transformStamped.transform);
}

visualization_msgs::Marker CartesianImpl::makeSTL( visualization_msgs::InteractiveMarker &msg )
{
  auto distal_link = getDistalLink();
  auto link = urdf_.getLink(distal_link);
  auto controlled_link = link;

  while(link && !link->visual)
  {
    if(!link->parent_joint)
    {
      ROS_WARN_STREAM("Unable to find mesh for link " << distal_link.c_str());
      return makeSphere(msg);
    }
    link = urdf_.getLink(link->parent_joint->parent_link_name);
  }

  if(!link) return makeSphere(msg);

  Eigen::Affine3d actual_pose = getPoseTF(controlled_link->name, link->name);
  affine3dToVisualizationPose(actual_pose, marker_);

  marker_.color.r = 0.5;
  marker_.color.g = 0.5;
  marker_.color.b = 0.5;

  if(link->visual->geometry->type == urdf::Geometry::MESH)
  {
    marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    auto mesh = std::static_pointer_cast<urdf::Mesh>(link->visual->geometry);
    marker_.mesh_resource = mesh->filename;
    marker_.scale.x = mesh->scale.x;
    marker_.scale.y = mesh->scale.y;
    marker_.scale.z = mesh->scale.z;
  }
  else if(link->visual->geometry->type == urdf::Geometry::BOX)
  {
    marker_.type = visualization_msgs::Marker::CUBE;
    auto box = std::static_pointer_cast<urdf::Box>(link->visual->geometry);
    marker_.scale.x = box->dim.x;
    marker_.scale.y = box->dim.y;
    marker_.scale.z = box->dim.z;
  }
  else if(link->visual->geometry->type == urdf::Geometry::CYLINDER)
  {
    marker_.type = visualization_msgs::Marker::CYLINDER;
    auto cyl = std::static_pointer_cast<urdf::Cylinder>(link->visual->geometry);
    marker_.scale.x = marker_.scale.y = cyl->radius;
    marker_.scale.z = cyl->length;
  }
  else if(link->visual->geometry->type == urdf::Geometry::SPHERE)
  {
    marker_.type = visualization_msgs::Marker::SPHERE;
    auto sph = std::static_pointer_cast<urdf::Sphere>(link->visual->geometry);
    marker_.scale.x = marker_.scale.y = marker_.scale.z = 2.*sph->radius;
  }

  marker_.color.a = .9;
  return marker_;
}

void CartesianImpl::makeMenu()
{
  continuous_control_entry_ =
      menu_handler_.insert("Marker Ctrl",
                           boost::bind(&CartesianImpl::setContinuousCtrl, this, _1));
  menu_handler_.setCheckState(continuous_control_entry_, interactive_markers::MenuHandler::CHECKED);

  way_point_entry_ = menu_handler_.insert("Add WayPoint");
  menu_handler_.setVisible(way_point_entry_, false);

  T_entry_ = menu_handler_.insert(way_point_entry_, "T [sec]");
  for (unsigned int i = 0; i < 10; i++ )
  {
    std::ostringstream sec_opt;
    double sec = (i+1) * 0.5;
    sec_opt << sec;
    T_last_ = menu_handler_.insert(T_entry_, sec_opt.str(),
                                   boost::bind(&CartesianImpl::wayPointCallBack, this, _1, sec));
    menu_handler_.setCheckState(T_last_,interactive_markers::MenuHandler::UNCHECKED);
  }

  reset_all_way_points_entry_ = menu_handler_.insert(way_point_entry_, "Reset All",
                                                     boost::bind(&CartesianImpl::resetAllWayPoints,this,_1));
  reset_lastway_point_entry_ = menu_handler_.insert(way_point_entry_, "Reset Last",
                                                    boost::bind(&CartesianImpl::resetLastWayPoints,this,_1));
  send_way_points_entry_ = menu_handler_.insert(way_point_entry_, "Send",
                                                boost::bind(&CartesianImpl::sendWayPoints,this,_1));

  base_link_entry_ = menu_handler_.insert("Base Link");
  menu_handler_.setVisible(base_link_entry_, true);

  std::string distal_link = getDistalLink();
  std::string base_link = getBaseLink();

  for(unsigned int i = 0; i < links_.size(); ++i)
  {
    if(distal_link != links_.at(i)->name)
    {
      interactive_markers::MenuHandler::EntryHandle link_entry =
          menu_handler_.insert(base_link_entry_, links_.at(i)->name,
                               boost::bind(&CartesianImpl::changeBaseLink, this, _1, links_.at(i)->name));

      if(base_link.compare("world") == 0 && (links_.at(i)->name).compare("world") == 0)
        menu_handler_.setCheckState(link_entry, interactive_markers::MenuHandler::CHECKED );
      else if(base_link.compare(links_.at(i)->name) == 0)
        menu_handler_.setCheckState(link_entry, interactive_markers::MenuHandler::CHECKED );
      else
        menu_handler_.setCheckState(link_entry, interactive_markers::MenuHandler::UNCHECKED );

      map_link_entries_[links_.at(i)->name] = link_entry;
    }
  }

  menu_control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  menu_control_.always_visible = true;

  interactive_marker_.controls.push_back(menu_control_);
  menu_handler_.apply(interactive_marker_server_, interactive_marker_.name);
}

void CartesianImpl::changeBaseLink(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                                   std::string new_base_link)
{
  int entry_id = feedback->menu_entry_id;
  menu_handler_.getTitle(entry_id, new_base_link);

  if(new_base_link.compare(getBaseLink()) == 0)
    return;

  if(setBaseLink(new_base_link))
  {
    reset();

    for (auto& tmp_map : map_link_entries_)
    {
      if(tmp_map.first == new_base_link)
        menu_handler_.setCheckState(tmp_map.second, interactive_markers::MenuHandler::CHECKED );
      else
        menu_handler_.setCheckState(tmp_map.second, interactive_markers::MenuHandler::UNCHECKED );
    }

    menu_handler_.reApply(interactive_marker_server_);
    interactive_marker_server_.applyChanges();
  }
}

void CartesianImpl::sendWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& /*feedback*/)
{
  if(waypoints_.empty()) return;

  std::partial_sum(T_.begin(), T_.end(), T_.begin());

  CartesianTrajectory::WayPointVector wpv;
  for(int i = 0; i < (int)waypoints_.size(); i++)
  {
    CartesianTrajectory::WayPoint wp;
    wp.T_ref = wolf_controller_utils::poseToAffine3d(waypoints_[i]);
    wp.duration = T_.at(i);
    wpv.push_back(wp);
  }

  trj_->setWayPoints(wpv);
  publishWP(waypoints_);

  waypoints_.clear();
  T_.clear();
}

void CartesianImpl::resetMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& /*feedback*/)
{
  clearMarker();
  spawnMarker();
}

void CartesianImpl::resetLastWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& /*feedback*/)
{
  ROS_INFO("Reset last waypoint!");

  if(!T_.empty())
    T_.pop_back();
  if(!waypoints_.empty())
    waypoints_.pop_back();

  clearMarker();

  if(waypoints_.empty())
    spawnMarker();
  else
  {
    if(interactive_marker_server_.empty())
    {
      poseToVisualizationPose(waypoints_.back(), interactive_marker_);
      interactive_marker_server_.insert(interactive_marker_,
                                        boost::bind(&CartesianImpl::processFeedback, this, _1));
      menu_handler_.reApply(interactive_marker_server_);
      interactive_marker_server_.applyChanges();
    }
  }

  publishWP(waypoints_);
}

void CartesianImpl::resetAllWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  T_.clear();
  waypoints_.clear();
  resetMarker(feedback);
  publishWP(waypoints_);
  ROS_INFO("Reset all waypoints!");
}

void CartesianImpl::wayPointCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, double T)
{
  double x,y,z,qx,qy,qz,qw;
  x  = feedback->pose.position.x;
  y  = feedback->pose.position.y;
  z  = feedback->pose.position.z;
  qx = feedback->pose.orientation.x;
  qy = feedback->pose.orientation.y;
  qz = feedback->pose.orientation.z;
  qw = feedback->pose.orientation.w;

  if(is_continuous_ == false)
  {
    ROS_INFO("\n %s set waypoint wrt %s @: \n pos = [%f, %f, %f],\n orient = [%f, %f, %f, %f],\n of %.1f secs",
             interactive_marker_.name.c_str(),
             getBaseLink().c_str(),
             x,y,z,
             qx,qy,qz,qw, T);

    waypoints_.push_back(feedback->pose);
    T_.push_back(T);
    publishWP(waypoints_);
  }
}

void CartesianImpl::publishWP(const std::vector<geometry_msgs::Pose>& wps)
{
  geometry_msgs::PoseArray msg;
  for(unsigned int i = 0; i < wps.size(); ++i)
    msg.poses.push_back(wps[i]);

  msg.header.frame_id = getBaseLink();
  msg.header.stamp = ros::Time::now();

  waypoints_pub_.publish(msg);
}

bool CartesianImpl::clearMarker()
{
  if(!interactive_marker_server_.empty())
  {
    interactive_marker_server_.erase(interactive_marker_.name);
    interactive_marker_server_.applyChanges();
  }
  return true;
}

bool CartesianImpl::spawnMarker()
{
  if(interactive_marker_server_.empty())
  {
    Eigen::Affine3d start_pose;
    getActualPose(start_pose);
    affine3dToVisualizationPose(start_pose, interactive_marker_);
    interactive_marker_server_.insert(interactive_marker_,
                                      boost::bind(&CartesianImpl::processFeedback, this, _1));
    menu_handler_.reApply(interactive_marker_server_);
    interactive_marker_server_.applyChanges();
  }
  return true;
}

void CartesianImpl::setContinuousCtrl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& /*feedback*/)
{
  is_continuous_ = !is_continuous_;

  if(is_continuous_ == false)
  {
    menu_handler_.setCheckState(continuous_control_entry_, interactive_markers::MenuHandler::UNCHECKED);
    menu_handler_.setVisible(way_point_entry_, true);
    waypoints_.clear();
    T_.clear();
    trj_->reset();
  }
  else
  {
    menu_handler_.setCheckState(continuous_control_entry_, interactive_markers::MenuHandler::CHECKED);
    menu_handler_.setVisible(way_point_entry_, false);
    waypoints_.clear();
    T_.clear();

    Eigen::Affine3d pose;
    Eigen::Matrix<double,6,1> twist = Eigen::Matrix<double,6,1>::Zero();
    getActualPose(pose);

    buffer_reference_pose_.writeFromNonRT(pose);
    buffer_reference_twist_.writeFromNonRT(twist);
  }

  clearMarker();
  spawnMarker();
  publishWP(waypoints_);

  menu_handler_.reApply(interactive_marker_server_);
  interactive_marker_server_.applyChanges();
}
