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
  std::string node_name_prefix;
  if(!robot_name.empty())
    node_name_prefix = robot_name+"/";
  else
    node_name_prefix = robot_name;

  //nh_= std::make_shared<rclcpp::Node>(node_name_prefix+"wolf_controller");
  task_nh_= std::make_shared<rclcpp::Node>(node_name_prefix+"wolf_controller_"+task_name);

  RCLCPP_INFO(task_nh_->get_logger(),"Created TaskRosHandler for %s",task_nh_->get_name());

  //ddr_server_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(this);
  auto pub = task_nh_->create_publisher<Msg_type>(task_name, 4);
  rt_pub_ = std::make_shared<realtime_tools::RealtimePublisher<Msg_type>>(pub);

  // Start the async spinner in a separate thread
  spinner_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  spinner_->add_node(task_nh_->get_node_base_interface());

  // Run the spinner in a new thread
  spinner_thread_ = std::make_shared<std::thread>([this]() {
    spinner_->spin();
  });
}

template<class Msg_type>
// In the destructor, don't forget to stop the thread and shutdown the node
TaskRosHandler<Msg_type>::~TaskRosHandler()
{
  // Shutdown the node and join the spinner thread
  if (spinner_thread_ && spinner_thread_->joinable()) {
    spinner_->cancel();
    spinner_thread_->join();
  }
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
