/**
 * WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola
 *
 * OpenSoT-free WBID orchestrator:
 *  - uses IDVariables for variable layout
 *  - each task provides (A,b,W) for least-squares terms
 *  - constraints contribute:
 *      * linear constraints (A,lA,uA) through IConstraint
 *      * variable bounds (l,u) through ConstraintBase (dynamic_cast)
 */

#pragma once
#ifndef WOLF_WBID_ID_PROBLEM_H
#define WOLF_WBID_ID_PROBLEM_H

// STD
#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Dense>

// WoLF
#include <wolf_wbid/core/quadruped_robot.h>
#include <wolf_controller_utils/common.h>
#include <wolf_wbid/wbid/id_variables.h>
#include <wolf_wbid/wbid/qp/qp_problem.h>
#include <wolf_wbid/wbid/qp/qp_solver.h>
#include <wolf_wbid/wbid/tasks/task_base.h>
#include <wolf_wbid/wbid/constraints/constraint_base.h>
#include <wolf_wbid/wbid/constraints/friction_cone_constraint.h>
#include <wolf_wbid/wbid/constraints/contact_force_bounds_constraint.h>
#include <wolf_wbid/wbid/constraints/torque_limits_constraint.h>
#include <wolf_wbid/wbid/constraints/dynamics_equality_constraint.h>
#include <wolf_wbid/wbid/constraints/base_accel_z_constraint.h>
#include <wolf_wbid/wbid/constraints/floating_base_accel_constraint.h>
#include <wolf_wbid/core/task_interface.h>

namespace wolf_wbid {

class IDProblem
{
public:
  enum mode_t { WPG = 0, EXT, MPC };

  const std::string CLASS_NAME = "IDProblem";

  using Ptr = std::shared_ptr<IDProblem>;
  using UniquePtr = std::unique_ptr<IDProblem>;
  

  explicit IDProblem(QuadrupedRobot::Ptr model);
  ~IDProblem();

  void init(const std::string& robot_name, const double& dt);

  bool solve(Eigen::VectorXd& tau);

  void enableDebug(bool enable) { debug_enabled_ = enable; }
  void setDebugMask(uint32_t mask) { debug_mask_ = mask; }

  enum DebugMask : uint32_t {
    DBG_NONE = 0,
    DBG_STATE = 1<<0,
    DBG_QP_BOUNDS = 1<<1,
    DBG_SOLUTION = 1<<2,
    DBG_TAU = 1<<3,
    DBG_CONTACTS = 1<<4,
    DBG_TASKS = 1<<5,
    DBG_CONSTRAINTS = 1<<6,
    DBG_ALL = 0xFFFFFFFF
  };

  const std::vector<Eigen::Vector6d>& getContactWrenches() const;
  const Eigen::VectorXd& getJointAccelerations() const;

  void swingWithFoot(const std::string& foot_name, const std::string& ref_frame);
  void stanceWithFoot(const std::string& foot_name, const std::string& ref_frame);

  void publish();
  void reset();

  // refs setters
  void setFootReference(const std::string& foot_name,
                        const Eigen::Affine3d& pose_ref,
                        const Eigen::Vector6d& vel_ref,
                        const std::string& reference_frame);

  void setWaistReference(const Eigen::Matrix3d& Rot, const double& z, const double& z_vel);
  void setComReference(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);

  void setPosture(const Eigen::MatrixXd& Kp, const Eigen::MatrixXd& Kd, const Eigen::VectorXd& q);

  // constraints/params
  void setFrictionConesMu(const double& mu);
  void setFrictionConesR(const Eigen::Matrix3d& R);
  double getFrictionConesMu() const;

  void setLowerForceBound(const double& x_force, const double& y_force, const double& z_force);
  void setLowerForceBoundX(const double& force);
  void setLowerForceBoundY(const double& force);
  void setLowerForceBoundZ(const double& force);

  void activateComZ(bool active);
  void activateAngularMomentum(bool active);
  void activatePostural(bool active);
  void activateJointPositionLimits(bool active); // placeholder for later

  void setRegularization(double regularization);
  void setForcesMinimizationWeight(double weight);
  void setJointAccelerationMinimizationWeight(double weight);

  // access tasks
  const std::map<std::string,Cartesian::Ptr>& getFootTasks() const;
  const std::map<std::string,Cartesian::Ptr>& getArmTasks() const;
  const std::map<std::string,Wrench::Ptr>& getWrenchTasks() const;
  const Cartesian::Ptr& getWaistTask() const;
  const Com::Ptr& getComTask() const;

  void setControlMode(mode_t mode); // kept for compatibility (WPG only used)

  void setSolver(std::unique_ptr<IQPSolver> solver);
  bool setSolverByName(const std::string& name);

  // Align cartesian references (feet/arms/waist) to current pose after transitions
  void resetCartesianReferences();


private:

  bool debug_enabled_{false};
  uint32_t debug_mask_{DBG_ALL};

  void debugDumpSolveStep(const QPProblem* qp,
                          const Eigen::VectorXd* x,
                          const Eigen::VectorXd* tau,
                          bool qp_built,
                          bool qp_solved,
                          bool torque_ok) const;

  static void debugPrintVecStats(const std::string& name, const Eigen::VectorXd& v);
  static void debugPrintMatStats(const std::string& name, const Eigen::MatrixXd& M);
  static void debugPrintBoundsStats(const std::string& name,
                                   const Eigen::VectorXd& l,
                                   const Eigen::VectorXd& u);

  void activateExternalReferences(bool activate);

  void update();                       // tasks update + update constraint params
  bool buildQP(QPProblem& qp);         // tasks -> H,g ; constraints -> bounds + A,lA,uA
  bool solveQP(IQPSolver& solver, const QPProblem& qp, Eigen::VectorXd& x);
  bool computeTauFromSolution(const Eigen::VectorXd& x, Eigen::VectorXd& tau);

  // objective helpers
  void addLeastSquaresTerm(QPProblem& qp,
                           const Eigen::MatrixXd& A,
                           const Eigen::VectorXd& b,
                           const Eigen::VectorXd& w_diag);

  void addLeastSquaresRows(QPProblem& qp,
                           const Eigen::MatrixXd& A,
                           const Eigen::VectorXd& b,
                           const Eigen::VectorXd& w_diag,
                           const std::vector<int>& rows);

  // constraints helper (supports bounds via dynamic_cast to ConstraintBase)
  void applyConstraintContributions(QPProblem& qp,
                                    const std::vector<std::shared_ptr<IConstraint>>& constraints);

  void setDefaultBounds(QPProblem& qp);

  // tasks
  std::map<std::string,Cartesian::Ptr> feet_;
  std::map<std::string,Cartesian::Ptr> arms_;
  std::map<std::string,Wrench::Ptr>   wrenches_;
  Cartesian::Ptr waist_;
  Com::Ptr com_;
  AngularMomentum::Ptr angular_momentum_;
  Postural::Ptr postural_;

  // model
  QuadrupedRobot::Ptr model_;

  // variables
  std::unique_ptr<IDVariables> vars_;

  // constraints per mode
  std::vector<std::shared_ptr<IConstraint>> constraints_;      // WPG stack
  std::vector<std::shared_ptr<IConstraint>> constraints_ext_;  // EXT stack

  // per-contact constraints for toggling
  std::map<std::string, std::shared_ptr<FrictionConeConstraint>> friction_cones_;
  std::map<std::string, std::shared_ptr<ContactForceBoundsConstraint>> force_bounds_;
  std::shared_ptr<TorqueLimitsConstraint> torque_limits_;
  std::shared_ptr<DynamicsEqualityConstraint> dynamics_eq_;
  std::shared_ptr<BaseAccelZConstraint> base_accel_z_;
  std::shared_ptr<FloatingBaseAccelConstraint> base_accel_fb_;

  // qp + solver
  QPProblem qp_;
  std::unique_ptr<IQPSolver> solver_;


  // solution buffers
  Eigen::VectorXd x_;
  Eigen::VectorXd qddot_;
  std::vector<Eigen::Vector6d> contact_wrenches_;

  // limits
  Eigen::Vector6d wrench_upper_lims_;
  Eigen::Vector6d wrench_lower_lims_;

  double x_force_lower_lim_{-2000.0};
  double y_force_lower_lim_{-2000.0};
  double z_force_lower_lim_{0.0};

  double mu_{1.0};
  Eigen::Matrix3d fc_R_{Eigen::Matrix3d::Identity()};

  // flags
  std::atomic<unsigned int> control_mode_{WPG};
  bool initialized_{false};

  bool activate_com_z_{true};
  bool activate_angular_momentum_{true};
  bool activate_postural_{false};
  bool activate_joint_position_limits_{false};

  bool change_control_mode_{false};

  // weights
  double regularization_value_{1e-3};
  double min_forces_weight_{0.0};
  double min_qddot_weight_{0.0};

  // names
  std::vector<std::string> foot_names_;
  std::vector<std::string> ee_names_;
  std::vector<std::string> contact_names_;

  // contact enable state
  std::map<std::string, bool> contact_enabled_;

  // temp
  Eigen::Affine3d tmp_affine3d_{Eigen::Affine3d::Identity()};
  Eigen::Vector6d tmp_vector6d_{Eigen::Vector6d::Zero()};
  Eigen::Vector3d tmp_vector3d_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tmp_vector3d_1_{Eigen::Vector3d::Zero()};
};

} // namespace wolf_wbid

#endif // WOLF_WBID_ID_PROBLEM_H
