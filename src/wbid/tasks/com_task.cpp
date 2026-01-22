#include <wolf_wbid/wbid/tasks/com_task.h>

#include <wolf_wbid/quadruped_robot.h>
#include <wolf_wbid/wbid/id_variables.h>

#include <stdexcept>
#include <cmath>

namespace wolf_wbid {

ComTask::ComTask(const std::string& task_id,
                 QuadrupedRobot& robot,
                 const IDVariables& vars)
  : task_id_(task_id),
    robot_(robot),
    vars_(vars)
{
  // Pre-allocate sizes
  // Jcom_ is 3 x dof_count (robot joint num + floating base dofs if present in model interface)
  // Here we just let the API fill it; but we must size A_ with vars_.size()
  A_.setZero(3, vars_.size());
  b_.setZero(3);

  // Initialize ref from current state
  resetReference();
}

void ComTask::setLambda(double lambda)
{
  if(lambda < 0.0) { throw std::invalid_argument("ComTask::setLambda(): lambda < 0"); }
  lambda1_ = lambda;
  lambda2_ = 2.0 * std::sqrt(lambda1_);
}

void ComTask::setLambda(double lambda1, double lambda2)
{
  if(lambda1 < 0.0 || lambda2 < 0.0)
  {
    throw std::invalid_argument("ComTask::setLambda(lambda1,lambda2): negative gain");
  }
  lambda1_ = lambda1;
  lambda2_ = lambda2;
}

void ComTask::setReference(const Eigen::Vector3d& pos_ref)
{
  pos_ref_ = pos_ref;
  vel_ref_.setZero();
  acc_ref_.setZero();

  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
}

void ComTask::setReference(const Eigen::Vector3d& pos_ref,
                           const Eigen::Vector3d& vel_ref)
{
  pos_ref_ = pos_ref;
  vel_ref_ = vel_ref;
  acc_ref_.setZero();

  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
}

void ComTask::setReference(const Eigen::Vector3d& pos_ref,
                           const Eigen::Vector3d& vel_ref,
                           const Eigen::Vector3d& acc_ref)
{
  pos_ref_ = pos_ref;
  vel_ref_ = vel_ref;
  acc_ref_ = acc_ref;

  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
}

void ComTask::resetReference()
{
  // QuadrupedRobot inherits ModelInterfaceRBDL -> ModelInterface
  // So getCOM() / getCOMVelocity() are available
  robot_.getCOM(pos_ref_);
  vel_ref_.setZero();
  acc_ref_.setZero();

  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;
}

bool ComTask::reset()
{
  resetReference();
  return true;
}

void ComTask::update(const Eigen::VectorXd& x)
{
  // cache feedforward terms (match OpenSoT behavior)
  vel_ref_cached_ = vel_ref_;
  acc_ref_cached_ = acc_ref_;

  // pull qddot from x
  const Eigen::VectorXd qddot = vars_.qddot(x);

  // get CoM Jacobian and bias term dJcomQdot
  // Signature you showed:
  // void ModelInterface::getCOMJacobian(Eigen::MatrixXd &J, Eigen::Vector3d &dJcomQdot) const
  robot_.getCOMJacobian(Jcom_, dJcomQdot_);

  // actual CoM and velocity
  robot_.getCOM(pos_current_);
  robot_.getCOMVelocity(vel_current_);

  // errors
  pos_error_ = pos_ref_ - pos_current_;
  vel_error_ = vel_ref_ - vel_current_;

  // cartesian acceleration term: Jcom*qddot + dJcomQdot
  //Eigen::Vector3d com_acc = Jcom_ * qddot + dJcomQdot_;

  // desired com acceleration y (OpenSoT sign convention):
  // OpenSoT builds: cart_task = J*qddot + dJdq - acc_ref - lambda2*Kd*(vel_ref - vel) - lambda*Kp*pos_err
  // Then A = cart_task.getM(), b = -cart_task.getq()  -> equivalent to minimizing J*qddot ~= acc_ref + ...
  Eigen::Vector3d y = acc_ref_
                    + lambda2_ * (Kd_ * vel_error_)
                    + lambda1_ * (Kp_ * pos_error_);

  // Build A,b in terms of full x: only qddot block is involved
  A_.setZero(3, vars_.size());
  A_.block(0, vars_.qddotBlock().offset, 3, vars_.qddotBlock().dim) = Jcom_;

  // residual: J*qddot + dJdq ~= y  => J*qddot ~= y - dJdq
  b_ = y - dJcomQdot_;

  // reset feed-forward for safety (OpenSoT behavior)
  vel_ref_.setZero();
  acc_ref_.setZero();
}

} // namespace wolf_wbid
