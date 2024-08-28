/**
 * @file handler.cpp
 * @author
 * @date 24 April, 2023
 * @brief This file contains the ROS2 handler
 */

#include <wolf_wbid/task_ros2_wrappers/handler.h>

using namespace wolf_wbid;

template<class Msg_type>
TaskRosHandler<Msg_type>::TaskRosHandler(const std::string& task_name, const std::string& robot_name, const double& period)
{
  nh_= std::make_shared<rclcpp::Node>(robot_name+"/wolf_controller");
  task_nh_= std::make_shared<rclcpp::Node>(robot_name+"/wolf_controller/"+task_name);

  //ddr_server_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(this);
  auto pub = nh_->create_publisher<Msg_type>(task_name, 4);
  rt_pub_ = std::make_shared<realtime_tools::RealtimePublisher<Msg_type>>(pub);
  // Start the async spinner
  spinner_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  spinner_->add_node(nh_->get_node_base_interface());
  spinner_->spin();
}

template<class Msg_type>
TaskRosHandler<Msg_type>::~TaskRosHandler()
{
  spinner_->cancel();
}

#include <wolf_msgs/msg/cartesian_task.hpp>
#include <wolf_msgs/msg/wrench_task.hpp>
#include <wolf_msgs/msg/com_task.hpp>
#include <wolf_msgs/msg/postural_task.hpp>

// Explicit template instantiation
template class wolf_wbid::TaskRosHandler<wolf_msgs::msg::CartesianTask>;
template class wolf_wbid::TaskRosHandler<wolf_msgs::msg::WrenchTask>;
template class wolf_wbid::TaskRosHandler<wolf_msgs::msg::ComTask>;
template class wolf_wbid::TaskRosHandler<wolf_msgs::msg::PosturalTask>;
