/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#pragma once
#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

#include <wolf_controller_utils/common.h>

namespace wolf_wbid {

class QuadrupedRobot;

class IDVariables
{
public:
  enum class ContactModel { POINT_CONTACT = 0, SURFACE_CONTACT = 1 };

  struct Block { int offset{0}; int dim{0}; };

  IDVariables(int joint_num,
              const std::vector<std::string>& contacts,
              ContactModel cm);

  int size() const { return size_; }
  int jointNum() const { return joint_num_; }
  int contactDim() const { return (cm_ == ContactModel::SURFACE_CONTACT) ? 6 : 3; }
  ContactModel contactModel() const { return cm_; }

  const std::vector<std::string>& contacts() const { return contacts_; }

  const Block& qddotBlock() const { return qddot_block_; }
  const Block& contactBlock(const std::string& name) const;

  bool hasContact(const std::string& name) const;

  Eigen::Map<const Eigen::VectorXd> qddot(const Eigen::VectorXd& x) const;
  Eigen::Map<Eigen::VectorXd> qddot(Eigen::VectorXd& x) const;

  int contactOffset(const std::string& name) const;

  // Convention: wrench = [Fx,Fy,Fz,Mx,My,Mz] (in base/world as defined by your Jacobian)
  Eigen::Vector3d contactForce3(const Eigen::VectorXd& x, const std::string& name) const;
  Eigen::Vector6d contactWrench6(const Eigen::VectorXd& x, const std::string& name) const;

  void unpack(const Eigen::VectorXd& x,
              Eigen::VectorXd& qddot_out,
              std::vector<Eigen::Vector6d>& wrenches_out) const;

  bool computeTorque(QuadrupedRobot& model,
                     const Eigen::VectorXd& x,
                     Eigen::VectorXd& tau_out,
                     Eigen::VectorXd& qddot_out,
                     std::vector<Eigen::Vector6d>& contact_wrenches_out) const;

private:
  int joint_num_{0};
  ContactModel cm_{ContactModel::POINT_CONTACT};
  std::vector<std::string> contacts_;

  Block qddot_block_;
  std::map<std::string, Block> blocks_;
  int size_{0};
};

} // namespace wolf_wbid
