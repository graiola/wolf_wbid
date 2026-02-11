/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#pragma once
#ifndef WOLF_WBID_COM_TASK_H
#define WOLF_WBID_COM_TASK_H

#include <Eigen/Dense>
#include <string>

#include <wolf_wbid/wbid/tasks/task_base.h>
#include <wolf_wbid/wbid/id_variables.h>

namespace wolf_wbid {

class QuadrupedRobot;

/**
 * @brief CoM acceleration task (3D):
 *
 * Model:
 *   pdd_com = Jcom * qddot + Jdotcom*qd
 *
 * LSQ:
 *   A = [Jcom, 0]
 *   b = pdd_des - Jdotcom*qd
 *
 * pdd_des = Kp*(p_ref - p) + Kd*(v_ref - v)
 */
class ComTask : public TaskBase
{
public:
  ComTask(const std::string& task_id,
          QuadrupedRobot& robot,
          const IDVariables& vars);

  const std::string& getBaseLink() const { return base_link_; }
  void setBaseLink(const std::string& base) { base_link_ = base; } // kept for msg frame compat

  // reference API
  void setReference(const Eigen::Vector3d& p_ref, const Eigen::Vector3d& v_ref);
  void getReference(Eigen::Vector3d& p_ref_out) const { p_ref_out = p_ref_; }
  const Eigen::Vector3d& getCachedVelocityReference() const { return v_ref_; }

  // actual getters used by wrappers
  void getActualPose(Eigen::Vector3d& p_act_out) const { p_act_out = p_act_; }
  void getActualVelocity(Eigen::Vector3d& v_act_out) const { v_act_out = v_act_; }

  const Eigen::Vector3d& posError() const { return e_p_; }
  const Eigen::Vector3d& velError() const { return e_v_; }

  void update(const Eigen::VectorXd& x) override;
  bool reset() override;

  // 3x3 gains helpers
  void setKp(const Eigen::Matrix3d& Kp3) { TaskBase::setKp(Kp3); }
  void setKd(const Eigen::Matrix3d& Kd3) { TaskBase::setKd(Kd3); }

protected:
  // adapt to your QuadrupedRobot API
  virtual void getCOM(Eigen::Vector3d& p_W) const;
  virtual void getCOMVelocity(Eigen::Vector3d& v_W) const;
  virtual void getCOMJacobian(Eigen::MatrixXd& Jcom) const;
  virtual void getCOMJacobianDotTimesQdot(Eigen::Vector3d& Jdot_qdot) const;

private:
  QuadrupedRobot& robot_;
  const IDVariables& vars_;

  std::string base_link_{WORLD_FRAME_NAME};

  IDVariables::Block qb_;

  // reference
  Eigen::Vector3d p_ref_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_ref_{Eigen::Vector3d::Zero()};

  // actual
  Eigen::Vector3d p_act_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_act_{Eigen::Vector3d::Zero()};

  // errors
  Eigen::Vector3d e_p_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d e_v_{Eigen::Vector3d::Zero()};
};

} // namespace wolf_wbid

#endif // WOLF_WBID_COM_TASK_H
