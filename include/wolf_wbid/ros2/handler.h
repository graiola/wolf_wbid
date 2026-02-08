/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef TASK_ROS2_WRAPPERS_HANDLER_H
#define TASK_ROS2_WRAPPERS_HANDLER_H

// ROS2
#include <rclcpp/rclcpp.hpp>
//#include <ddynamic_reconfigure/ddynamic_reconfigure.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>


// STD
#include <memory>
#include <thread>

// WoLF
#include <wolf_wbid/core/task_interface.h>

namespace wolf_wbid {

template<class Msg_type>
class TaskRosHandler
{
public:
  using Ptr = std::shared_ptr<TaskRosHandler>;

  TaskRosHandler(const std::string& task_name, const std::string& robot_name, const double& period);

  ~TaskRosHandler();

protected:

  //std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<rclcpp::Node> task_nh_;

  //std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_server_;
  std::shared_ptr<realtime_tools::RealtimePublisher<Msg_type>> rt_pub_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> spinner_;
  std::shared_ptr<std::thread> spinner_thread_;
};

} // namespace wolf_wbid

#endif // TASK_ROS2_WRAPPERS_HANDLER_H
