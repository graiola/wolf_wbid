/**
WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef TASK_ROS2_WRAPPERS_CARTESIAN_H
#define TASK_ROS2_WRAPPERS_CARTESIAN_H

// ROS2
#include <urdf/model.h>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <interactive_markers/menu_handler.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

// WoLF
#include <wolf_wbid/task_ros2_wrappers/handler.h>
#include <wolf_wbid/cartesian_trajectory.h>

// WoLF utils
#include <wolf_controller_utils/geometry.h>
#include <wolf_controller_utils/converters.h>

// WoLF msgs
#include <wolf_msgs/msg/cartesian_task.hpp>
#include <wolf_msgs/msg/cartesian.hpp>

// STD
#include <numeric>

namespace wolf_wbid {

// CARTESIAN
class CartesianImpl : public Cartesian, public TaskRosHandler<wolf_msgs::msg::CartesianTask>
{

public:

  using Ptr = std::shared_ptr<CartesianImpl>;

  CartesianImpl(const std::string& robot_name,
            const std::string& task_id,
            const XBot::ModelInterface& robot,
            const std::string& distal_link,
            const std::string& base_link,
            const OpenSoT::AffineHelper& qddot,
            const double& period = 0.001,
            const bool& use_mesh = true);

  virtual void registerReconfigurableVariables() override;

  virtual void loadParams() override;

  virtual void updateCost(const Eigen::VectorXd& x) override;

  virtual void publish() override;

  virtual bool reset() override;

protected:

  void makeMarker(const std::string &distal_link, const std::string &base_link, unsigned int interaction_mode, bool show);

  void createInteractiveMarkerControl(const double qw, const double qx, const double qy, const double qz,
                                      const unsigned int interaction_mode);

  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

  void referenceCallback(const wolf_msgs::msg::Cartesian::SharedPtr msg);

  visualization_msgs::msg::InteractiveMarkerControl& makeSTLControl(visualization_msgs::msg::InteractiveMarker& msg);

  visualization_msgs::msg::Marker makeSphere(visualization_msgs::msg::InteractiveMarker& msg);

  visualization_msgs::msg::Marker makeSTL( visualization_msgs::msg::InteractiveMarker &msg );

  Eigen::Affine3d getPose(const std::string& base_link, const std::string& distal_link);

  void makeMenu();

  void changeBaseLink(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, std::string new_base_link);

  void sendWayPoints(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

  void resetMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

  void resetLastWayPoints(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

  void resetAllWayPoints(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

  void wayPointCallBack(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback, double T);

  void publishWP(const std::vector<geometry_msgs::msg::Pose>& wps);

  bool clearMarker();

  bool spawnMarker();

  void setContinuousCtrl(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

private:

  virtual void _update(const Eigen::VectorXd& x) override;

  /**
   * @brief waypoints_pub_ ROS2 publisher for the waypoint poses
   */
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;

  /**
   * @brief is_continuous_ True if the poses are directly given to the task
   */
  bool is_continuous_;

  /**
   * @brief waypoints_ Stores all the waypoints, excluding the initial position
   */
  std::vector<geometry_msgs::msg::Pose> waypoints_;

  /**
   * @brief T_ Stores the time of each waypoint in the trajectory
   */
  std::vector<float> T_;

  int control_type_;

  std::shared_ptr<std_srvs::srv::Empty::Request> req_;
  std::shared_ptr<std_srvs::srv::Empty::Response> res_;

  /**
   * @brief Marker variables
   */
  std::string marker_name_;
  visualization_msgs::msg::InteractiveMarker interactive_marker_;
  interactive_markers::InteractiveMarkerServer interactive_marker_server_;
  visualization_msgs::msg::Marker marker_;
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
  visualization_msgs::msg::InteractiveMarkerControl menu_control_;
  std::map<std::string, interactive_markers::MenuHandler::EntryHandle> map_link_entries_;

  /**
   * @brief urdf_ model description of the robot
   */
  urdf::ModelInterface urdf_;

  /**
   * @brief links_ Available URDF links
   */
  std::vector<urdf::LinkSharedPtr> links_;

  /**
   * @brief use_mesh_ True if the end-effector mesh is used for marker visualization
   */
  bool use_mesh_;

  /**
   * @brief tf_buffer_ TF2 buffer for transform lookup
   */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief listener_ TF2 transform listener
   */
  std::shared_ptr<tf2_ros::TransformListener> listener_;

  /**
   * @brief trj_ Cartesian trajectory interpolator
   */
  CartesianTrajectory::Ptr trj_;

  /**
   * @brief Realtime buffers for reference pose and twist
   */
  realtime_tools::RealtimeBuffer<Eigen::Affine3d> buffer_reference_pose_;
  realtime_tools::RealtimeBuffer<Eigen::Vector6d> buffer_reference_twist_;

  typename rclcpp::Subscription<wolf_msgs::msg::Cartesian>::SharedPtr reference_sub_;

};

} // namespace wolf_wbid

#endif // TASK_ROS2_WRAPPERS_CARTESIAN_H
