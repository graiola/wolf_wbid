/**
WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
*/

#ifndef WOLF_WBID_TASK_INTERFACE_H
#define WOLF_WBID_TASK_INTERFACE_H

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// STD
#include <atomic>
#include <memory>
#include <string>

// WoLF utils
#include <wolf_controller_utils/common.h>

// Forward declarations
namespace wolf_wbid {
class QuadrupedRobot;
class IDVariables;
}

#define WORLD_FRAME_NAME "world" // FIXME REMOVE!

// New OpenSoT-free tasks
#include <wolf_wbid/wbid/tasks/cartesian_task.h>
#include <wolf_wbid/wbid/tasks/com_task.h>
#include <wolf_wbid/wbid/tasks/angular_momentum_task.h>
#include <wolf_wbid/wbid/tasks/postural_task.h>
#include <wolf_wbid/wbid/tasks/wrench_task.h>

namespace wolf_wbid {

/**
 * Base interface for ROS wrappers (ROS1/ROS2).
 * IMPORTANT: do NOT include ROS headers here.
 *
 * This class defines the *single* stable entrypoint the solver will call: update(x).
 * It uses the Template Method pattern:
 *   update(x) = applyExternalKnobs() + applyExternalReference() + doUpdate(x)
 */
class TaskWrapperInterface
{
public:
  using Ptr = std::shared_ptr<TaskWrapperInterface>;

  struct {
    std::atomic<bool> set_ext_lambda    = true;
    std::atomic<bool> set_ext_weight    = true;
    std::atomic<bool> set_ext_gains     = true;
    std::atomic<bool> set_ext_reference = false;
  } OPTIONS;

  TaskWrapperInterface(const std::string& task_name, // TODO Remove task name, use task id
                       const std::string& robot_name,
                       const double& period);

  virtual ~TaskWrapperInterface() = default;

  // Solver entrypoint (single, deterministic)
  void update(const Eigen::VectorXd& x);

  // Wrapper API (no ROS headers here)
  virtual void publish() = 0;
  virtual void loadParams() = 0;
  virtual void registerReconfigurableVariables() {}
  virtual void updateCost(const Eigen::VectorXd& x) = 0;

  // DDR callbacks (keep legacy API) -> they only write buffers
  void setLambda1(double value);
  void setLambda2(double value);
  void setWeightDiag(double value);

  void setKpX(double value);
  void setKpY(double value);
  void setKpZ(double value);
  void setKpRoll(double value);
  void setKpPitch(double value);
  void setKpYaw(double value);

  void setKdX(double value);
  void setKdY(double value);
  void setKdZ(double value);
  void setKdRoll(double value);
  void setKdPitch(double value);
  void setKdYaw(double value);

protected:
  // Hooks called by update(x)
  // Default: no-op. Concrete wrappers (ROS handlers) override what they need.
  virtual void applyExternalKnobs() {}
  virtual void applyExternalReference() {}

  // Task-core update (A,b,errors,...). Must be implemented by the stable handle classes.
  virtual void doUpdate(const Eigen::VectorXd& x) = 0;

protected:
  double      period_{0.001};
  std::string task_name_;
  std::string robot_name_;

  // Scratch buffers
  Eigen::VectorXd    tmp_vectorXd_;
  Eigen::Affine3d    tmp_affine3d_{Eigen::Affine3d::Identity()};
  Eigen::Vector6d    tmp_vector6d_{Eigen::Vector6d::Zero()};
  Eigen::Vector6d    tmp_vector6d_1_{Eigen::Vector6d::Zero()};
  Eigen::Vector3d    tmp_vector3d_{Eigen::Vector3d::Zero()};
  Eigen::Matrix6d    tmp_matrix6d_{Eigen::Matrix6d::Zero()};
  Eigen::Matrix3d    tmp_matrix3d_{Eigen::Matrix3d::Zero()};
  Eigen::Quaterniond tmp_quaterniond_{Eigen::Quaterniond::Identity()};

  // External buffers (DDR writes these)
  std::atomic<double> buffer_lambda1_{0.0};
  std::atomic<double> buffer_lambda2_{0.0};
  std::atomic<double> buffer_weight_diag_{1.0};

  std::atomic<double> buffer_kp_x_{0.0};
  std::atomic<double> buffer_kp_y_{0.0};
  std::atomic<double> buffer_kp_z_{0.0};
  std::atomic<double> buffer_kp_roll_{0.0};
  std::atomic<double> buffer_kp_pitch_{0.0};
  std::atomic<double> buffer_kp_yaw_{0.0};

  std::atomic<double> buffer_kd_x_{0.0};
  std::atomic<double> buffer_kd_y_{0.0};
  std::atomic<double> buffer_kd_z_{0.0};
  std::atomic<double> buffer_kd_roll_{0.0};
  std::atomic<double> buffer_kd_pitch_{0.0};
  std::atomic<double> buffer_kd_yaw_{0.0};

  double last_time_{0.0};
  double cost_{0.0};
};

// --------------------
// Stable task handles used by IDProblem
// Each handle:
//  - exposes a single update(x) (hides the task-base update to avoid ambiguity)
//  - implements doUpdate(x) by calling the task-core update
//  - can implement common applyExternalKnobs() if desired (later)
// --------------------

// --------------------
// CARTESIAN
// --------------------
class Cartesian : public CartesianTask, public TaskWrapperInterface
{
public:
  using Ptr = std::shared_ptr<Cartesian>;

  Cartesian(const std::string& robot_name,
            const std::string& task_id,
            QuadrupedRobot& robot,
            const std::string& distal_link,
            const std::string& base_link,
            const IDVariables& vars,
            const double& period = 0.001,
            const bool& /*use_mesh*/ = true)
  : CartesianTask(task_id, robot, distal_link, base_link, vars)
  , TaskWrapperInterface(task_id, robot_name, period)
  {}

  // IMPORTANT: expose a single update(x) to the solver
  void update(const Eigen::VectorXd& x) { TaskWrapperInterface::update(x); }

  virtual bool reset() = 0;

protected:
  void doUpdate(const Eigen::VectorXd& x) final { CartesianTask::update(x); }
};

// --------------------
// COM
// --------------------
class Com : public ComTask, public TaskWrapperInterface
{
public:
  using Ptr = std::shared_ptr<Com>;

  Com(const std::string& robot_name,
      const std::string& task_id,
      QuadrupedRobot& robot,
      const IDVariables& vars,
      const double& period = 0.001)
  : ComTask(task_id, robot, vars)
  , TaskWrapperInterface(task_id, robot_name, period)
  {}

  void update(const Eigen::VectorXd& x) { TaskWrapperInterface::update(x); }

  virtual bool reset() = 0;

protected:
  void doUpdate(const Eigen::VectorXd& x) final { ComTask::update(x); }
};

// --------------------
// WRENCH (point contact force R^3)
// --------------------
class Wrench : public WrenchTask, public TaskWrapperInterface
{
public:
  using Ptr = std::shared_ptr<Wrench>;

  Wrench(const std::string& robot_name,
         const std::string& task_id,
         const std::string& contact_name,
         const double& period = 0.001,
         double weight = 1.0)
  : WrenchTask(task_id, contact_name, weight)
  , TaskWrapperInterface(task_id, robot_name, period)
  {}

  void update(const Eigen::VectorXd& x) { TaskWrapperInterface::update(x); }

  virtual bool reset() = 0;

protected:
  void doUpdate(const Eigen::VectorXd& x) final { WrenchTask::update(x); }
};

class AngularMomentum : public AngularMomentumTask, public TaskWrapperInterface
{
public:
  using Ptr = std::shared_ptr<AngularMomentum>;

  AngularMomentum(const std::string& robot_name,
                  QuadrupedRobot& robot,
                  const IDVariables& vars,
                  const std::string& task_id = "angular_momentum",
                  const double& period = 0.001)
  : AngularMomentumTask(task_id, robot, vars)
  , TaskWrapperInterface(task_id, robot_name, period)
  {}

  void update(const Eigen::VectorXd& x) { TaskWrapperInterface::update(x); }

  virtual bool reset() = 0;

protected:
  void doUpdate(const Eigen::VectorXd& x) final { AngularMomentumTask::update(x); }
};

class Postural : public PosturalTask, public TaskWrapperInterface
{
public:
  using Ptr = std::shared_ptr<Postural>;

  Postural(const std::string& robot_name,
           QuadrupedRobot& robot,
           const IDVariables& vars,
           const std::string& task_id = "postural",
           const double& period = 0.001)
  : PosturalTask(task_id, robot, vars)
  , TaskWrapperInterface(task_id, robot_name, period)
  {}

  void update(const Eigen::VectorXd& x) { TaskWrapperInterface::update(x); }

  virtual bool reset() = 0;

protected:
  void doUpdate(const Eigen::VectorXd& x) final { PosturalTask::update(x); }
};

} // namespace wolf_wbid

#endif // WOLF_WBID_TASK_INTERFACE_H
