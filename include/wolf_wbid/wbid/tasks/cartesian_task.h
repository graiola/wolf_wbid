/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#pragma once
#ifndef WOLF_WBID_CARTESIAN_TASK_H
#define WOLF_WBID_CARTESIAN_TASK_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

#include <wolf_wbid/wbid/tasks/task_base.h>
#include <wolf_wbid/wbid/id_variables.h>

namespace wolf_wbid {

class QuadrupedRobot;

/**
 * @brief 6D Cartesian task at acceleration level:
 *
 * Variables: x = [qddot; f_contact...]
 *
 * Model:
 *   xdd_task = J(q) * qddot + Jdot(q,qd) * qd   (6x1)
 *
 * We build LSQ:
 *   A = [ J , 0 ]
 *   b = xdd_des - Jdot*qd
 *
 * Desired xdd_des is computed from pose+twist references with PD:
 *   pos_err (6) from SE(3) logarithm (approx)
 *   vel_err (6) = v_ref - v_act
 *   xdd_des = Kp * pos_err + Kd * vel_err
 *
 * GainType:
 *  - Acceleration: uses PD as above
 *  - Force: currently uses the same acceleration-level PD.
 *    It can be replaced later with an impedance-to-wrench mapping if needed.
 */
class CartesianTask : public TaskBase
{
public:
  enum class GainType { Acceleration = 0, Force = 1 };

  CartesianTask(const std::string& task_id,
                QuadrupedRobot& robot,
                const std::string& distal_link,
                const std::string& base_link,
                const IDVariables& vars);

  // identity
  const std::string& getDistalLink() const { return distal_link_; }
  const std::string& getBaseLink()   const { return base_link_;  }

  bool setBaseLink(const std::string& new_base_link);
  void setGainType(GainType t) { gain_type_ = t; }
  GainType getGainType() const { return gain_type_; }

  // reference API used by wrappers
  void setReference(const Eigen::Affine3d& pose_ref,
                    const Eigen::Matrix<double,6,1>& twist_ref = Eigen::Matrix<double,6,1>::Zero());

  void getReference(Eigen::Affine3d& pose_ref_out) const { pose_ref_out = pose_ref_; }
  const Eigen::Matrix<double,6,1>& getCachedVelocityReference() const { return twist_ref_; }

  // actual getters used by wrappers
  void getActualPose(Eigen::Affine3d& pose_act_out) const { pose_act_out = pose_act_; }
  void getActualTwist(Eigen::Matrix<double,6,1>& twist_act_out) const { twist_act_out = twist_act_; }

  // errors used by wrappers
  const Eigen::Matrix<double,6,1>& getError() const { return e6_pos_; }
  const Eigen::Matrix<double,6,1>& getVelocityError() const { return e6_vel_; }

  // update
  void update() override;
  bool reset() override;

  // convenience: set Kp/Kd as 6x6
  void setKp(const Eigen::Matrix<double,6,6>& Kp6) { TaskBase::setKp(Kp6); }
  void setKd(const Eigen::Matrix<double,6,6>& Kd6) { TaskBase::setKd(Kd6); }

protected:
  // Adapt these hooks if your QuadrupedRobot backend uses different APIs.
  virtual void getJacobian6(const std::string& link, Eigen::MatrixXd& J6) const;
  virtual void getJacobianDotTimesQdot6(const std::string& link,
                                        Eigen::Matrix<double,6,1>& Jdot_qdot) const;

  virtual void getLinkPoseInWorld(const std::string& link, Eigen::Affine3d& T_W_L) const;

  // utilities
  static Eigen::Matrix<double,6,1> se3LogApprox(const Eigen::Affine3d& T_err);

private:
  QuadrupedRobot& robot_;
  const IDVariables& vars_;

  std::string distal_link_;
  std::string base_link_;
  GainType gain_type_{GainType::Acceleration};

  // cached block info
  IDVariables::Block qb_;

  // reference
  Eigen::Affine3d pose_ref_{Eigen::Affine3d::Identity()};
  Eigen::Matrix<double,6,1> twist_ref_{Eigen::Matrix<double,6,1>::Zero()};

  // actual (computed at update)
  Eigen::Affine3d pose_act_{Eigen::Affine3d::Identity()};
  Eigen::Matrix<double,6,1> twist_act_{Eigen::Matrix<double,6,1>::Zero()};

  // errors
  Eigen::Matrix<double,6,1> e6_pos_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> e6_vel_{Eigen::Matrix<double,6,1>::Zero()};

  // preallocated runtime buffers (RT-safe update path)
  Eigen::MatrixXd J6_;
  Eigen::MatrixXd Bi_;
  Eigen::MatrixXd tmp6xn_;
};

} // namespace wolf_wbid

#endif // WOLF_WBID_CARTESIAN_TASK_H
