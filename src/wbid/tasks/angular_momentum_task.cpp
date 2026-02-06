#include <wolf_wbid/wbid/tasks/angular_momentum_task.h>
#include <wolf_wbid/quadruped_robot.h>

#include <stdexcept>

namespace wolf_wbid {

AngularMomentumTask::AngularMomentumTask(const std::string& task_id,
                                         QuadrupedRobot& robot,
                                         const IDVariables& vars)
: TaskBase(task_id)
, robot_(robot)
, vars_(vars)
{
  qb_ = vars_.qddotBlock();
  if(qb_.dim <= 0) throw std::runtime_error("AngularMomentumTask: invalid qddot block");

  // LSQ: 3 rows (angular part only), nvars cols
  resize(3, vars_.size());

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

  K_.setIdentity();

  is_init_ = false;
}

void AngularMomentumTask::setMomentumGain(const Eigen::Matrix3d& K)
{
  for(int i = 0; i < 3; ++i) {
    if(!std::isfinite(K(i,i)) || K(i,i) < 0.0)
      throw std::runtime_error("AngularMomentumTask::setMomentumGain(): invalid diag");
  }
  K_ = K;
}

void AngularMomentumTask::setReference(const Eigen::Vector3d& desiredAngularMomentum,
                                       const Eigen::Vector3d& desiredAngularMomentumVariation)
{
  L_d_    = desiredAngularMomentum;
  Ldot_d_ = desiredAngularMomentumVariation;
}

bool AngularMomentumTask::reset()
{
  is_init_ = false;

  L_d_.setZero();
  Ldot_d_.setZero();

  L_d_cached_.setZero();
  Ldot_d_cached_.setZero();

  Ldot_ref_.setZero();
  return true;
}

void AngularMomentumTask::update(const Eigen::VectorXd& /*x*/)
{
  if(!enabled()) {
    b_.setZero();
    return;
  }

  // Cache references (OpenSoT-like: what you set is visible for introspection/debug)
  L_d_cached_    = L_d_;
  Ldot_d_cached_ = Ldot_d_;

  // 1) Centroidal momentum matrix and bias term CMMdot*qdot
  robot_.getCentroidalMomentumMatrix(Mom_, CMMdotQdot_);

  // 2) Centroidal momentum h = [p; L] -> take angular part
  robot_.getCentroidalMomentum(h_);
  L_ang_ = h_.tail<3>();

  // 3) Initialize reference once with current momentum (OpenSoT behavior)
  if(!is_init_) {
    L_d_ = L_ang_;
    is_init_ = true;
    L_d_cached_ = L_d_; // keep caches consistent on first iteration
  }

  // 4) Tracking law:
  //    Ldot_ref = Ldot_d + lambda * K * (L_d - L)
  //    Use TaskBase::lambda1_ as the legacy scalar lambda.
  const double lambda = lambda1_;
  Ldot_ref_ = Ldot_d_ + lambda * (K_ * (L_d_ - L_ang_));

  // 5) Task: Mom_bottom * qddot + bias_bottom ~= Ldot_ref
  Ldot_bias_ = CMMdotQdot_.tail<3>();

  A_.setZero();
  A_.block(0, qb_.offset, 3, qb_.dim) = Mom_.bottomRows(3);

  b_ = Ldot_ref_ - Ldot_bias_;

  // 6) One-shot feedforward reset (OpenSoT behavior)
  L_d_.setZero();
  Ldot_d_.setZero();
}

} // namespace wolf_wbid
