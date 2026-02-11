/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#pragma once
#ifndef WOLF_WBID_ANGULAR_MOMENTUM_TASK_H
#define WOLF_WBID_ANGULAR_MOMENTUM_TASK_H

#include <Eigen/Dense>
#include <string>

#include <wolf_wbid/wbid/tasks/task_base.h>
#include <wolf_wbid/wbid/id_variables.h>

namespace wolf_wbid {

class QuadrupedRobot;

/**
 * @brief Centroidal angular momentum rate tracking task (3D).
 *
 * Model:
 *   h = [p; L]
 *   Ldot = (CMM_bottom) * qddot + (CMMdot*qdot)_bottom
 *
 * Tracking law:
 *   Ldot_ref = Ldot_d + lambda * K * (L_d - L)
 *
 * LSQ term:
 *   A = CMM_bottom placed on qddot block
 *   b = Ldot_ref - (CMMdot*qdot)_bottom
 *
 * Update behavior:
 *  - first update() initializes L_d to current L
 *  - reference is cached for publishing
 *  - one-shot reset of L_d and Ldot_d after building the task
 */
class AngularMomentumTask : public TaskBase
{
public:
  AngularMomentumTask(const std::string& task_id,
                      QuadrupedRobot& robot,
                      const IDVariables& vars);

  // Lambda helper stored in TaskBase::lambda1_.
  void setLambda(double lambda) { TaskBase::setLambda(lambda, 0.0); }

  // Reference interface.
  void setReference(const Eigen::Vector3d& desiredAngularMomentum);
  void setReference(const Eigen::Vector3d& desiredAngularMomentum,
                    const Eigen::Vector3d& desiredAngularMomentumVariation);

  void getReference(Eigen::Vector3d& desiredAngularMomentum) const;
  void getReference(Eigen::Vector3d& desiredAngularMomentum,
                    Eigen::Vector3d& desiredAngularMomentumVariation) const;

  // Momentum gain K (3x3, usually diagonal)
  void setMomentumGain(const Eigen::Matrix3d& K);
  const Eigen::Matrix3d& getMomentumGain() const { return K_; }

  // Helpers for wrappers / debug
  const Eigen::Vector3d& getActualAngularMomentum() const { return L_ang_; }
  const Eigen::Vector3d& getCachedDesiredAngularMomentum() const { return L_d_cached_; }
  const Eigen::Vector3d& getCachedDesiredAngularMomentumRate() const { return Ldot_d_cached_; }

  void update() override;
  bool reset() override;

private:
  QuadrupedRobot& robot_;
  const IDVariables& vars_;
  IDVariables::Block qb_;

  // Model buffers.
  Eigen::MatrixXd Mom_;          // 6 x nj
  Eigen::Vector6d CMMdotQdot_;   // 6
  Eigen::Vector6d h_;            // 6

  Eigen::Vector3d L_ang_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d Ldot_bias_{Eigen::Vector3d::Zero()};

  // References and cached values.
  Eigen::Vector3d L_d_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d Ldot_d_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d L_d_cached_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d Ldot_d_cached_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d Ldot_ref_{Eigen::Vector3d::Zero()};

  Eigen::Matrix3d K_{Eigen::Matrix3d::Identity()};

  bool is_init_{false};
};

} // namespace wolf_wbid

#endif // WOLF_WBID_ANGULAR_MOMENTUM_TASK_H
