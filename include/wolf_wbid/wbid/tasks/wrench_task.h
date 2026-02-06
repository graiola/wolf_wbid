#pragma once
#ifndef WOLF_WBID_WRENCH_TASK_H
#define WOLF_WBID_WRENCH_TASK_H

#include <Eigen/Dense>
#include <string>

#include <wolf_wbid/wbid/tasks/task_base.h>
#include <wolf_wbid/wbid/id_variables.h>

namespace wolf_wbid {

/**
 * @brief POINT_CONTACT wrench tracking task (force only).
 *
 * Variables block: f = [Fx,Fy,Fz] at contactBlock(contact_name).
 * Residual: r = f - f_ref
 * So:
 *   A is (3 x nvars) selecting the contact force block
 *   b is (3) equal to f_ref
 * wDiag is (3) usually constant diag = weight
 */
class WrenchTask : public TaskBase
{
public:
  WrenchTask(const std::string& task_id,
             const std::string& contact_name,
             const IDVariables& vars,
             double weight_scalar = 1.0);

  const std::string& contactName() const { return contact_name_; }

  // reference API
  void setReference(const Eigen::Vector3d& f_ref);
  const Eigen::Vector3d& reference() const { return f_ref_; }

  // scalar weight helper (legacy wrappers use this)
  double weight() const { return getWeightScalar(); }
  void setWeight(double w) { setWeightScalar(w); }

  void update(const Eigen::VectorXd& x) override;
  bool reset() override;

private:
  std::string contact_name_;
  const IDVariables& vars_;

  Eigen::Vector3d f_ref_{Eigen::Vector3d::Zero()};

  // Cached block info
  IDVariables::Block cb_;
};

} // namespace wolf_wbid

#endif // WOLF_WBID_WRENCH_TASK_H
