#include <wolf_wbid/wbid/id_variables.h>
#include <wolf_wbid/core/quadruped_robot.h>
#include <stdexcept>
#include <cmath>

namespace wolf_wbid {

IDVariables::IDVariables(int joint_num,
                         const std::vector<std::string>& contacts,
                         ContactModel cm)
: joint_num_(joint_num), cm_(cm), contacts_(contacts)
{
  int off = 0;
  qddot_block_ = {off, joint_num_};
  off += joint_num_;

  const int cd = contactDim();
  for(const auto& c : contacts_){
    blocks_.emplace(c, Block{off, cd});
    off += cd;
  }
  size_ = off;
}

const IDVariables::Block& IDVariables::contactBlock(const std::string& name) const
{
  auto it = blocks_.find(name);
  if(it == blocks_.end()) throw std::runtime_error("Unknown contact block: " + name);
  return it->second;
}

bool IDVariables::hasContact(const std::string& name) const
{
  return blocks_.find(name) != blocks_.end();
}

Eigen::Map<const Eigen::VectorXd> IDVariables::qddot(const Eigen::VectorXd& x) const
{
  return Eigen::Map<const Eigen::VectorXd>(x.data() + qddot_block_.offset, qddot_block_.dim);
}

Eigen::Map<Eigen::VectorXd> IDVariables::qddot(Eigen::VectorXd& x) const
{
  return Eigen::Map<Eigen::VectorXd>(x.data() + qddot_block_.offset, qddot_block_.dim);
}

int IDVariables::contactOffset(const std::string& name) const
{
  return contactBlock(name).offset;
}

Eigen::Vector3d IDVariables::contactForce3(const Eigen::VectorXd& x, const std::string& name) const
{
  const auto& b = contactBlock(name);

  // POINT_CONTACT: block is [Fx,Fy,Fz]
  // SURFACE_CONTACT: block is [Fx,Fy,Fz,Mx,My,Mz] -> return first 3 (force part)
  return Eigen::Map<const Eigen::Vector3d>(x.data() + b.offset);
}

Eigen::Vector6d IDVariables::contactWrench6(const Eigen::VectorXd& x, const std::string& name) const
{
  const auto& b = contactBlock(name);

  Eigen::Vector6d w; w.setZero();
  if(cm_ == ContactModel::SURFACE_CONTACT){
    w = Eigen::Map<const Eigen::Matrix<double,6,1>>(x.data() + b.offset);
  } else {
    w.head<3>() = Eigen::Map<const Eigen::Vector3d>(x.data() + b.offset);
  }
  return w;
}

void IDVariables::unpack(const Eigen::VectorXd& x,
                         Eigen::VectorXd& qddot_out,
                         std::vector<Eigen::Vector6d>& wrenches_out) const
{
  qddot_out = qddot(x);
  wrenches_out.resize(contacts_.size());
  for(size_t i=0;i<contacts_.size();++i){
    wrenches_out[i] = contactWrench6(x, contacts_[i]);
  }
}

bool IDVariables::computeTorque(QuadrupedRobot& model,
                               const Eigen::VectorXd& x,
                               Eigen::VectorXd& tau_out,
                               Eigen::VectorXd& qddot_out,
                               std::vector<Eigen::Vector6d>& contact_wrenches_out) const
{
  if(x.size() != size_) return false;

  qddot_out = qddot(x);

  model.setJointAcceleration(qddot_out);
  model.update();

  const int ndofs = model.getJointNum();
  if(tau_out.size() != ndofs) {
    tau_out.resize(ndofs);
  }
  model.computeInverseDynamics(tau_out); // assumes no contact forces inside model ID

  if(J_tmp_.rows() != 6 || J_tmp_.cols() != ndofs) {
    J_tmp_.resize(6, ndofs);
  }
  contact_wrenches_out.resize(contacts_.size());

  for(size_t i=0;i<contacts_.size();++i){
    const auto& link = contacts_[i];

    contact_wrenches_out[i] = contactWrench6(x, link);

    model.getJacobian(link, J_tmp_); // 6 x n
    tau_out.noalias() -= J_tmp_.transpose() * contact_wrenches_out[i];
  }

  if(model.isFloatingBase()){
    for(int i=0; i<6 && i<tau_out.size(); ++i){
      if(std::abs(tau_out[i]) > 1e-2){
        return false;
      }
    }
  }
  return true;
}

} // namespace wolf_wbid
