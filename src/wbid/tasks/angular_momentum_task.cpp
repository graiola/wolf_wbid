// ============================================================================
// File: src/wbid/tasks/angular_momentum_task.cpp
// ============================================================================

#include <wolf_wbid/wbid/tasks/angular_momentum_task.h>
#include <wolf_wbid/wbid/id_variables.h>
#include <wolf_wbid/quadruped_robot.h>

#include <cmath>

namespace wolf_wbid {

AngularMomentumTask::AngularMomentumTask(const std::string& task_id,
                                         QuadrupedRobot& robot,
                                         const IDVariables& vars)
  : task_id_(task_id),
    robot_(robot),
    vars_(vars)
{
  Mom_.setZero(6, robot_.getJointNum());
  CMMdotQdot_.setZero();

  h_.setZero();
  L_ang_.setZero();
  Ldot_bias_.setZero();

  L_d_.setZero();
  Ldot_d_.setZero();
  L_d_cached_.setZero();
  Ldot_d_cached_.setZero();
  Ldot_ref_.setZero();

  A_.setZero(3, vars_.size());
  b_.setZero(3);

  is_init_ = false;
}

void AngularMomentumTask::setLambda(double lambda)
{
  if(lambda < 0.0) return;
  lambda_ = lambda;
}

void AngularMomentumTask::setReference(const Eigen::Vector3d& desiredAngularMomentum)
{
  L_d_ = desiredAngularMomentum;
  Ldot_d_.setZero();
}

void AngularMomentumTask::setReference(const Eigen::Vector3d& desiredAngularMomentum,
                                       const Eigen::Vector3d& desiredAngularMomentumVariation)
{
  L_d_ = desiredAngularMomentum;
  Ldot_d_ = desiredAngularMomentumVariation;
}

void AngularMomentumTask::getReference(Eigen::Vector3d& desiredAngularMomentum) const
{
  desiredAngularMomentum = L_d_;
}

void AngularMomentumTask::getReference(Eigen::Vector3d& desiredAngularMomentum,
                                       Eigen::Vector3d& desiredAngularMomentumVariation) const
{
  desiredAngularMomentum = L_d_;
  desiredAngularMomentumVariation = Ldot_d_;
}

bool AngularMomentumTask::reset()
{
  is_init_ = false;
  L_d_.setZero();
  Ldot_d_.setZero();
  L_d_cached_.setZero();
  Ldot_d_cached_.setZero();
  return true;
}

void AngularMomentumTask::update(const Eigen::VectorXd& x)
{
  // Pull qddot from x
  const Eigen::VectorXd qddot = vars_.qddot(x);

  // 1) centroidal momentum matrix and bias
  robot_.getCentroidalMomentumMatrix(Mom_, CMMdotQdot_);

  // 2) centroidal momentum h = [p; L] from YOUR API
  robot_.getCentroidalMomentum(h_);
  L_ang_ = h_.tail<3>();

  // 3) init reference once (OpenSoT behavior)
  if(!is_init_)
  {
    L_d_ = L_ang_;
    is_init_ = true;
  }

  // 4) cache references (for publish)
  L_d_cached_ = L_d_;
  Ldot_d_cached_ = Ldot_d_;

  // 5) tracking law
  Ldot_ref_ = Ldot_d_ + lambda_ * (K_ * (L_d_ - L_ang_));

  // 6) task:
  //    Mom_bottom*qddot + bias_bottom ~= Ldot_ref
  Ldot_bias_ = CMMdotQdot_.tail<3>();

  A_.setZero();
  A_.block(0, vars_.qddotBlock().offset, 3, vars_.qddotBlock().dim) = Mom_.bottomRows(3);
  b_ = Ldot_ref_ - Ldot_bias_;

  // 7) safety reset (OpenSoT behavior)
  L_d_.setZero();
  Ldot_d_.setZero();
}

} // namespace wolf_wbid
