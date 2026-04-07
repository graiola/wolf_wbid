/**
 * @file handler.cpp
 * @author
 * @date 24 April, 2023
 * @brief This file contains the ROS2 handler
 */

#include <wolf_wbid/ros2/handler.h>
#include <wolf_controller_utils/namespace_utils.h>

using namespace wolf_wbid;

template<class Msg_type>
TaskRosHandler<Msg_type>::TaskRosHandler(const std::string& task_name, const std::string& robot_name, const double& period)
{
  task_nh_= std::make_shared<rclcpp::Node>(
      task_name,
      wolf_controller_utils::controller_namespace(robot_name));

  //ddr_server_ = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(this);
  // Use a plain ROS 2 publisher for task diagnostics. The Humble realtime publisher
  // path has proven unstable here and these topics are not control-critical.
  pub_ = task_nh_->create_publisher<Msg_type>(task_name, 4);

  // Prepare a dedicated executor, but do not spin it until the concrete task
  // has completed construction and parameter initialization.
  spinner_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  spinner_->add_node(task_nh_->get_node_base_interface());

  RCLCPP_INFO(task_nh_->get_logger(),"Created TaskRosHandler for %s",task_nh_->get_name());
}

template<class Msg_type>
void TaskRosHandler<Msg_type>::startSpinner()
{
  if (!spinner_)
    return;

  if (spinner_thread_ && spinner_thread_->joinable())
    return;

  spinner_thread_ = std::make_shared<std::thread>([spinner = spinner_]() {
    spinner->spin();
  });
}

template<class Msg_type>
void TaskRosHandler<Msg_type>::stopSpinner()
{
  if (spinner_)
    spinner_->cancel();

  if (spinner_thread_ && spinner_thread_->joinable())
    spinner_thread_->join();

  spinner_thread_.reset();
}

template<class Msg_type>
// In the destructor, don't forget to stop the thread and shutdown the node
TaskRosHandler<Msg_type>::~TaskRosHandler()
{
  stopSpinner();
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
