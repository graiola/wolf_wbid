/**
WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
**/

#ifndef TASK_ROS_WRAPPERS_CARTESIAN_H
#define TASK_ROS_WRAPPERS_CARTESIAN_H

// ROS
#include <interactive_markers/menu_handler.h>
#include <urdf/model.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// WoLF
#include <wolf_wbid/task_interface.h>                 // <- brings wolf_wbid::Cartesian handle
#include <wolf_wbid/task_ros_wrappers/handler.h>
#include <wolf_wbid/cartesian_trajectory.h>

// WoLF utils
#include <wolf_controller_utils/geometry.h>
#include <wolf_controller_utils/converters.h>

// WoLF msgs
#include <wolf_msgs/CartesianTask.h>
#include <wolf_msgs/Cartesian.h>

// STD
#include <numeric>
#include <map>
#include <sstream>

namespace wolf_wbid {

class QuadrupedRobot;
class IDVariables;

// CARTESIAN (ROS wrapper)
class CartesianImpl : public Cartesian, public TaskRosHandler<wolf_msgs::CartesianTask>
{
public:
  using Ptr = std::shared_ptr<CartesianImpl>;

  CartesianImpl(const std::string& robot_name,
                const std::string& task_id,
                QuadrupedRobot& robot,
                const std::string& distal_link,
                const std::string& base_link,
                const IDVariables& vars,
                const double& period = 0.001,
                const bool& use_mesh = true);

  void registerReconfigurableVariables() override;
  void loadParams() override;
  void updateCost(const Eigen::VectorXd& x) override;
  void publish() override;
  bool reset() override;

protected:

  void applyExternalKnobs() override;
  void applyExternalReference() override;

  // marker helpers
  void makeMarker(const std::string &distal_link, const std::string &base_link,
                  unsigned int interaction_mode, bool show);

  void createInteractiveMarkerControl(const double qw, const double qx, const double qy, const double qz,
                                      const unsigned int interaction_mode);

  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  void referenceCallback(const wolf_msgs::Cartesian::ConstPtr& msg);

  visualization_msgs::InteractiveMarkerControl& makeSTLControl(visualization_msgs::InteractiveMarker& msg);

  visualization_msgs::Marker makeSphere(visualization_msgs::InteractiveMarker& msg);

  visualization_msgs::Marker makeSTL( visualization_msgs::InteractiveMarker &msg );

  Eigen::Affine3d getPoseTF(const std::string& base_link, const std::string& distal_link);

  void makeMenu();

  void changeBaseLink(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                      std::string new_base_link);

  void sendWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void resetMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void resetLastWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void resetAllWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void wayPointCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, double T);

  void publishWP(const std::vector<geometry_msgs::Pose>& wps);

  bool clearMarker();

  bool spawnMarker();

  void setContinuousCtrl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

private:
  QuadrupedRobot& robot_;

  // ROS publisher with the waypoint poses
  ros::Publisher waypoints_pub_;

  // true if the poses are directly given to the task
  bool is_continuous_{true};

  // contains all the waypoints BUT not the initial position!
  std::vector<geometry_msgs::Pose> waypoints_;

  // contains the times of each waypoint-trajectory
  std::vector<float> T_;

  int control_type_{visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D};

  std_srvs::EmptyRequest req_;
  std_srvs::EmptyResponse res_;

  // Marker variables
  visualization_msgs::InteractiveMarker interactive_marker_;
  interactive_markers::InteractiveMarkerServer interactive_marker_server_;
  visualization_msgs::Marker marker_;
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler::EntryHandle reset_marker_entry_;
  interactive_markers::MenuHandler::EntryHandle way_point_entry_;
  interactive_markers::MenuHandler::EntryHandle base_link_entry_;
  interactive_markers::MenuHandler::EntryHandle T_entry_;
  interactive_markers::MenuHandler::EntryHandle T_last_;
  interactive_markers::MenuHandler::EntryHandle reset_lastway_point_entry_;
  interactive_markers::MenuHandler::EntryHandle reset_all_way_points_entry_;
  interactive_markers::MenuHandler::EntryHandle send_way_points_entry_;
  interactive_markers::MenuHandler::EntryHandle continuous_control_entry_;
  visualization_msgs::InteractiveMarkerControl  menu_control_;
  std::map<std::string,interactive_markers::MenuHandler::EntryHandle> map_link_entries_;

  // urdf model description of the robot
  urdf::ModelInterface urdf_;
  std::vector<urdf::LinkSharedPtr> links_;
  bool use_mesh_{true};

  // Cartesian trajectory interpolator
  CartesianTrajectory::Ptr trj_;

  // Realtime buffers
  realtime_tools::RealtimeBuffer<Eigen::Affine3d> buffer_reference_pose_;
  realtime_tools::RealtimeBuffer<Eigen::Matrix<double,6,1>> buffer_reference_twist_;
};

} // namespace wolf_wbid

#endif // TASK_ROS_WRAPPERS_CARTESIAN_H
