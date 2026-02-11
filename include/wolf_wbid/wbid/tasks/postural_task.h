/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#pragma once
#ifndef WOLF_WBID_POSTURAL_TASK_H
#define WOLF_WBID_POSTURAL_TASK_H

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <wolf_wbid/wbid/tasks/task_base.h>
#include <wolf_wbid/wbid/id_variables.h>

namespace wolf_wbid {

class QuadrupedRobot;

/**
 * @brief Joint-space postural acceleration tracking task:
 *
 * qdd_des = Kp*(q_ref - q) + Kd*(qd_ref - qd)
 *
 * LSQ:
 *   A = [I, 0]   over qddot block
 *   b = qdd_des
 *
 * This matches your wrappers that publish:
 *  actualQ, actualQdot, refQ, refQdotCached, errors
 *
 * weight handled as row weights (size = njoints).
 */
class PosturalTask : public TaskBase
{
public:
  PosturalTask(const std::string& task_id,
               QuadrupedRobot& robot,
               const IDVariables& vars);

  // legacy-style knobs used in your wrappers
  double getLambda1() const { return TaskBase::getLambda1(); }
  double getLambda2() const { return TaskBase::getLambda2(); }

  double getWeightDiag() const { return (wDiag().size() ? wDiag()(0) : 0.0); }
  void setWeightDiag(double w); // sets scalar and row weights constant

  // reference & gains
  void setReference(const Eigen::VectorXd& q_ref);
  void setReference(const Eigen::VectorXd& q_ref, const Eigen::VectorXd& qd_ref);

  void setGains(const Eigen::MatrixXd& Kp, const Eigen::MatrixXd& Kd);

  // accessors for wrapper publishing
  int taskSize() const { return n_; }

  const std::vector<std::string>& jointNames() const { return joint_names_; }

  const Eigen::VectorXd& actualQ() const { return q_act_; }
  const Eigen::VectorXd& actualQdot() const { return qd_act_; }

  const Eigen::VectorXd& refQ() const { return q_ref_; }
  const Eigen::VectorXd& refQdotCached() const { return qd_ref_; }

  const Eigen::VectorXd& posError() const { return e_q_; }
  const Eigen::VectorXd& velError() const { return e_qd_; }

  // optional helper used by your wrapper (computeCost(x))
  double computeCost(const Eigen::VectorXd& x) const;

  void update(const Eigen::VectorXd& x) override;
  bool reset() override;

protected:
  // adapt to your QuadrupedRobot API
  virtual void getJointPosition(Eigen::VectorXd& q) const;
  virtual void getJointVelocity(Eigen::VectorXd& qd) const;
  virtual std::vector<std::string> getJointNames() const;

private:
  QuadrupedRobot& robot_;
  const IDVariables& vars_;

  IDVariables::Block qb_;

  int n_{0};

  std::vector<std::string> joint_names_;

  Eigen::VectorXd q_act_;
  Eigen::VectorXd qd_act_;

  Eigen::VectorXd q_ref_;
  Eigen::VectorXd qd_ref_;

  Eigen::VectorXd e_q_;
  Eigen::VectorXd e_qd_;
};

} // namespace wolf_wbid

#endif // WOLF_WBID_POSTURAL_TASK_H
