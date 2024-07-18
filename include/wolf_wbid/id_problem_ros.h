/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef WOLF_WBID_ID_PROBLEM_ROS_H
#define WOLF_WBID_ID_PROBLEM_ROS_H

// WoLF ID problem
#include <wolf_wbid/wolf_id_problem.h>

// ROS
#include <ros/ros.h>

// WoLF
#include <wolf_wbid/ros_wrappers/momentum.h>
#include <wolf_wbid/ros_wrappers/postural.h>
#include <wolf_wbid/ros_wrappers/cartesian.h>
#include <wolf_wbid/ros_wrappers/com.h>
#include <wolf_wbid/ros_wrappers/wrench.h>
#include <wolf_wbid/quadruped_robot.h>

// WoLF utils
#include <wolf_controller_utils/geometry.h>

// STD
#include <atomic>
#include <mutex>
#include <memory>

namespace wolf_wbid{

/**
 * @brief The IDProblemRos class wraps the tasks, constraints and the solver used to solve the inverse dynamic problem (ID)
 */
class IDProblemRos : public IDProblem
{

public:

    const std::string CLASS_NAME = "IDProblemRos";

    typedef std::shared_ptr<IDProblemRos> Ptr;

    typedef std::unique_ptr<IDProblemRos> UniquePtr;

    /**
     * @brief IDProblemRos constructor
     * @param model pointer to external model
     */
    IDProblemRos(QuadrupedRobot::Ptr model);

    /**
     * @brief IDProblemRos destructor
     */
    ~IDProblemRos();

    /**
     * @brief initialize the IDProblem
     * @param robot_name
     * @param dt
     */
    virtual void init(const std::string& robot_name, const double& dt);

    /**
     * @brief publish the ros topics related to the tasks
     * @param ros current time
     * @param ros current period
     */
    void publish(const ros::Time& time, const ros::Duration& period);

};

} // namespace

#endif

