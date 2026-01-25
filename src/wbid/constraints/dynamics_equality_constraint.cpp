#include <wolf_wbid/wbid/constraints/dynamics_equality_constraint.h>

#include <algorithm>
#include <stdexcept>

namespace wolf_wbid {

static inline void setEqRows(Eigen::VectorXd& lA, Eigen::VectorXd& uA, const Eigen::VectorXd& b)
{
  lA = b;
  uA = b;
}

DynamicsEqualityConstraint::DynamicsEqualityConstraint(const std::string& name,
                                                       QuadrupedRobot& robot,
                                                       const IDVariables& vars,
                                                       const std::vector<std::string>& contact_links)
: ConstraintBase(name),
  robot_(robot),
  vars_(vars),
  contacts_(contact_links)
{
  if(vars_.contactDim() != 3)
    throw std::runtime_error("DynamicsEqualityConstraint: implemented only for POINT_CONTACT (dim=3)");

  enabled_.assign(contacts_.size(), true);

  // 6 equality rows, n vars columns
  resizeLinear(6, vars_.size());

  // We'll set lA=uA every update
  lA_.setZero(6);
  uA_.setZero(6);
}

int DynamicsEqualityConstraint::idxOf(const std::string& contact_link) const
{
  auto it = std::find(contacts_.begin(), contacts_.end(), contact_link);
  if(it == contacts_.end()) return -1;
  return static_cast<int>(std::distance(contacts_.begin(), it));
}

bool DynamicsEqualityConstraint::enableContact(const std::string& contact_link)
{
  int i = idxOf(contact_link);
  if(i < 0) return false;
  enabled_[static_cast<size_t>(i)] = true;
  return true;
}

bool DynamicsEqualityConstraint::disableContact(const std::string& contact_link)
{
  int i = idxOf(contact_link);
  if(i < 0) return false;
  enabled_[static_cast<size_t>(i)] = false;
  return true;
}

/*void DynamicsEqualityConstraint::update(const Eigen::VectorXd& x)
{
  // 1) get B(q), h(q,qd)
  robot_.getInertiaMatrix(B_);
  robot_.computeNonlinearTerm(h_);

  if(B_.rows() < 6 || B_.cols() < 6 || h_.size() < 6)
    throw std::runtime_error("DynamicsEqualityConstraint: robot model seems not floating-base (missing 6 dofs)");

  Bu_ = B_.topRows(6);
  hu_ = h_.head(6);

  // 2) Build equality: Bu*qddot + hu + sum( -Jf * wrench_i ) = 0
  // => A*x = b with b = -hu, A = [Bu, (-Jf * S_point)].
  A_.setZero();

  const auto& qb = vars_.qddotBlock();
  A_.block(0, qb.offset, 6, qb.dim) = Bu_;

  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    if(!enabled_[i]) continue;

    const std::string& c = contacts_[i];
    if(!vars_.hasContact(c)) continue;

    robot_.getJacobian(c, Jtmp_); // 6 x n (world), like OpenSoT

    if(Jtmp_.rows() < 6 || Jtmp_.cols() < 6)
      throw std::runtime_error("DynamicsEqualityConstraint: Jacobian too small for contact " + c);

    //// OpenSoT:
    ////   _Jf = _Jtmp.block<6,6>(0,0).transpose();
    //Eigen::Matrix<double,6,6> Jf = Jtmp_.block<6,6>(0,0).transpose();

    //// POINT_CONTACT: wrench = [0; f] (assuming [tau; force])
    //// so (-Jf) * wrench = (-Jf.rightCols(3)) * f
    //Eigen::Matrix<double,6,3> Jf_point = Jf.rightCols<3>();   // if [tau;force]
    //// If your wrench ordering is [force;tau], then use leftCols<3>() instead:
    ////Eigen::Matrix<double,6,3> Jf_point = Jf.leftCols<3>();

    //const auto& cb = vars_.contactBlock(c); // dim=3
    //A_.block(0, cb.offset, 6, 3).noalias() += (-Jf_point);


    // OpenSoT-style Jf
    Eigen::Matrix<double,6,6> Jf = Jtmp_.block<6,6>(0,0).transpose();

    // POINT_CONTACT embedding: wrench6 = [f; 0]  (force first, torque last)
    Eigen::Matrix<double,6,3> S_point;
    S_point.setZero();
    S_point.block<3,3>(0,0).setIdentity();

    // Contribution: (-Jf) * (S_point * f) = (-(Jf*S_point)) * f
    Eigen::Matrix<double,6,3> Jf_point = -(Jf * S_point);

    const auto& cb = vars_.contactBlock(c); // dim=3
    A_.block(0, cb.offset, 6, 3).noalias() += Jf_point;
  }

  // b = -hu
  lA_ = -hu_;
  uA_ = -hu_;
}*/

void DynamicsEqualityConstraint::update(const Eigen::VectorXd& /*x*/)
{
  robot_.getInertiaMatrix(B_);
  robot_.computeNonlinearTerm(h_);

  if(B_.rows() < 6 || B_.cols() < 6 || h_.size() < 6)
    throw std::runtime_error("DynamicsEqualityConstraint: robot model seems not floating-base");

  // Base rows of full dynamics
  Bu_ = B_.topRows(6);
  hu_ = h_.head(6);

  A_.setZero();

  const auto& qb = vars_.qddotBlock();
  A_.block(0, qb.offset, 6, qb.dim) = Bu_;

  // For POINT_CONTACT we want: -(J_lin_base^T) * f
  // with J ordering [v;w] => linear is topRows(3)
  // base columns are the first 6 columns.
  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    if(!enabled_[i]) continue;

    const std::string& c = contacts_[i];
    if(!vars_.hasContact(c)) continue;

    robot_.getJacobian(c, Jtmp_); // 6 x ndofs

    if(Jtmp_.rows() < 6 || Jtmp_.cols() < 6)
      throw std::runtime_error("DynamicsEqualityConstraint: bad Jacobian size for " + c);

    const auto& cb = vars_.contactBlock(c); // dim=3

    // J_lin_base is 3x6 => transpose is 6x3
    A_.block(0, cb.offset, 6, 3).noalias() +=
        -Jtmp_.topRows(3).leftCols(6).transpose();
  }

  // Equality: A*x = -hu
  lA_ = -hu_;
  uA_ = -hu_;
}



} // namespace wolf_wbid

