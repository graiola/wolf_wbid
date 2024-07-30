/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef WOLF_WBID_TASK_INTERFACE_H
#define WOLF_WBID_TASK_INTERFACE_H

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// STD
#include <atomic>
#include <memory>

// WoLF
#include <wolf_controller_utils/common.h>

// OpenSoT
#include <OpenSoT/Task.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/tasks/acceleration/AngularMomentum.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/force/Force.h>

namespace wolf_wbid {

class TaskWrapperInterface
{

public:

  typedef std::shared_ptr<TaskWrapperInterface> Ptr;

  struct {
    std::atomic<bool> set_ext_lambda    = true;
    std::atomic<bool> set_ext_weight    = true;
    std::atomic<bool> set_ext_gains     = true;
    std::atomic<bool> set_ext_reference = false;
  } OPTIONS;

  TaskWrapperInterface(const std::string& task_name, const std::string& robot_name, const double& period);
  virtual ~TaskWrapperInterface();
  virtual void publish() = 0;

  virtual void loadParams() = 0;

  virtual void registerReconfigurableVariables() {};

  virtual void updateCost(const Eigen::VectorXd& x) = 0;

  void setLambda1(double value)    ;
  void setLambda2(double value)    ;
  void setWeightDiag(double value) ;

  void setKpX(double value)    ;
  void setKpY(double value)    ;
  void setKpZ(double value)    ;
  void setKpRoll(double value) ;
  void setKpPitch(double value);
  void setKpYaw(double value)  ;

  void setKdX(double value)    ;
  void setKdY(double value)    ;
  void setKdZ(double value)    ;
  void setKdRoll(double value) ;
  void setKdPitch(double value);
  void setKdYaw(double value)  ;

protected:

  double                period_;
  std::string           task_name_;
  std::string           robot_name_;

  Eigen::VectorXd       tmp_vectorXd_;
  Eigen::Affine3d       tmp_affine3d_;
  Eigen::Vector6d       tmp_vector6d_;
  Eigen::Vector6d       tmp_vector6d_1_;
  Eigen::Vector3d       tmp_vector3d_;
  Eigen::Matrix6d       tmp_matrix6d_;
  Eigen::Matrix3d       tmp_matrix3d_;
  Eigen::Quaterniond    tmp_quaterniond_;

  std::atomic<double> buffer_lambda1_;
  std::atomic<double> buffer_lambda2_;
  std::atomic<double> buffer_weight_diag_;

  std::atomic<double> buffer_kp_x_;
  std::atomic<double> buffer_kp_y_;
  std::atomic<double> buffer_kp_z_;
  std::atomic<double> buffer_kp_roll_;
  std::atomic<double> buffer_kp_pitch_;
  std::atomic<double> buffer_kp_yaw_;

  std::atomic<double> buffer_kd_x_;
  std::atomic<double> buffer_kd_y_;
  std::atomic<double> buffer_kd_z_;
  std::atomic<double> buffer_kd_roll_;
  std::atomic<double> buffer_kd_pitch_;
  std::atomic<double> buffer_kd_yaw_;

  double last_time_;

  double cost_;

};

// CARTESIAN
class Cartesian : public OpenSoT::tasks::acceleration::Cartesian, public TaskWrapperInterface
{

public:

  typedef std::shared_ptr<Cartesian> Ptr;

  Cartesian(const std::string& robot_name,
            const std::string& task_id,
            const XBot::ModelInterface& robot,
            const std::string& distal_link,
            const std::string& base_link,
            const OpenSoT::AffineHelper& qddot,
            const double& period = 0.001,
            const bool& use_mesh = true)
    :OpenSoT::tasks::acceleration::Cartesian(task_id,robot, distal_link,base_link,qddot)
    ,TaskWrapperInterface(task_id,robot_name,period){}

  virtual bool reset() = 0;

};

// Com
class Com : public OpenSoT::tasks::acceleration::CoM, public TaskWrapperInterface
{

public:

  typedef std::shared_ptr<Com> Ptr;

  Com(const std::string& robot_name,
      const XBot::ModelInterface& robot,
      const OpenSoT::AffineHelper& qddot,
      const double& period = 0.001)
    :OpenSoT::tasks::acceleration::CoM(robot,qddot)
    ,TaskWrapperInterface(_task_id,robot_name,period){}

  virtual bool reset() = 0;

};

// AngularMomentum
class AngularMomentum : public OpenSoT::tasks::acceleration::AngularMomentum, public TaskWrapperInterface
{

public:

  typedef std::shared_ptr<AngularMomentum> Ptr;

  AngularMomentum(const std::string& robot_name,
                  XBot::ModelInterface& robot,
                  const OpenSoT::AffineHelper& qddot,
                  const double& period = 0.001)
  :OpenSoT::tasks::acceleration::AngularMomentum(robot,qddot)
  ,TaskWrapperInterface(_task_id,robot_name,period){}

  virtual bool reset() = 0;

};

// POSTURAL
class Postural : public OpenSoT::tasks::acceleration::Postural, public TaskWrapperInterface
{

public:

  typedef std::shared_ptr<Postural> Ptr;

  Postural(const std::string& robot_name,
           const XBot::ModelInterface& robot,
           OpenSoT::AffineHelper qddot = OpenSoT::AffineHelper(),
           const std::string& task_id = "postural",
           const double& period = 0.001)
  :OpenSoT::tasks::acceleration::Postural(robot,qddot,task_id)
  ,TaskWrapperInterface(task_id,robot_name,period){}

  virtual bool reset() = 0;

};

// Wrench
class Wrench : public OpenSoT::tasks::force::Wrench, public TaskWrapperInterface
{

public:

  typedef std::shared_ptr<Wrench> Ptr;

  Wrench(const std::string& robot_name,
         const std::string& task_id,
         const std::string& distal_link,
         const std::string& base_link,
         OpenSoT::AffineHelper& wrench,
         const double& period = 0.001)
  :OpenSoT::tasks::force::Wrench(task_id,distal_link,base_link,wrench)
  ,TaskWrapperInterface(task_id,robot_name,period){}

  virtual bool reset() = 0;

};

} // namespace

#endif

