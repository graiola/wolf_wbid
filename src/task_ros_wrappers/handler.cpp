/**
 * @file handler.cpp
 * @author Gennaro Raiola
 * @date 24 April, 2023
 * @brief This file contains the ROS handler
 */

#include <wolf_wbid/task_ros_wrappers/handler.h>

using namespace wolf_wbid;

template<class Msg_type>
TaskRosHandler<Msg_type>::TaskRosHandler(const std::string& task_name, const std::string& robot_name, const double& period)
{
  spinner_.reset(new ros::AsyncSpinner(1));
  spinner_->start();

  nh_ = ros::NodeHandle(robot_name+"/wolf_controller");
  rt_pub_.reset(new realtime_tools::RealtimePublisher<Msg_type>(nh_,task_name, 4));
  task_nh_ = ros::NodeHandle(nh_.getNamespace()+"/"+task_name);
  ddr_server_.reset(new ddynamic_reconfigure::DDynamicReconfigure(task_nh_));
}

template<class Msg_type>
TaskRosHandler<Msg_type>::~TaskRosHandler()
{
   spinner_->stop();
}

#include <wolf_msgs/CartesianTask.h>
#include <wolf_msgs/WrenchTask.h>
#include <wolf_msgs/ComTask.h>
#include <wolf_msgs/PosturalTask.h>

// Explicit template instantiation
template class wolf_wbid::TaskRosHandler<wolf_msgs::CartesianTask>;
template class wolf_wbid::TaskRosHandler<wolf_msgs::WrenchTask>;
template class wolf_wbid::TaskRosHandler<wolf_msgs::ComTask>;
template class wolf_wbid::TaskRosHandler<wolf_msgs::PosturalTask>;
