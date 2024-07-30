/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef TASK_ROS_WRAPPERS_HANDLER_H
#define TASK_ROS_WRAPPERS_HANDLER_H

// ROS
#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <interactive_markers/interactive_marker_server.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <eigen_conversions/eigen_msg.h>

// STD
#include <memory>

// WoLF
#include <wolf_wbid/task_interface.h>

namespace wolf_wbid {

template<class Msg_type>
class TaskRosHandler
{

public:

  typedef std::shared_ptr<TaskRosHandler> Ptr;

  TaskRosHandler(const std::string& task_name, const std::string& robot_name, const double& period);

  ~TaskRosHandler();

protected:

  ros::NodeHandle       nh_;
  ros::NodeHandle       task_nh_;

  std::shared_ptr<ros::AsyncSpinner> spinner_;
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_server_;

  std::shared_ptr<realtime_tools::RealtimePublisher<Msg_type>> rt_pub_;
  ros::Subscriber reference_sub_;

};

} // namespace

#endif // ROS_WRAPPERS_INTERFACE_H

