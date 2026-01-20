#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <string>

#include <wolf_wbid/wbid/tasks/task_lsq.h>

namespace wolf_wbid {

class IDVariables;

/**
 * POINT_CONTACT ONLY (R^3): minimize || f_contact - f_ref ||_W
 * Decision variable block for this contact is (fx, fy, fz).
 *
 * NOTE:
 *  - No ROS I/O here (handled by wrapper)
 *  - No thread-safety / mutex here (aligned with other tasks)
 *  - No caching of actual/cost here (wrapper can compute from solution)
 */
class WrenchTask : public ITaskLSQ
{
public:
  using Ptr = std::shared_ptr<WrenchTask>;

  WrenchTask(std::string task_name,
             std::string contact_name,
             double weight = 1.0);

  std::string name() const override { return task_name_; }
  const std::string& contactName() const { return contact_name_; }

  // Weight (diagonal, same scalar for x/y/z)
  void setWeight(double w);
  double weight() const { return weight_.load(); }

  // Reference force (fx,fy,fz)
  void setReference(const Eigen::Vector3d& f_ref);
  const Eigen::Vector3d& reference() const { return f_ref_; }

  // ITaskLSQ
  void compute(const IDVariables& vars, LsqTerm& out) override;

  // Reset policy: by default reset ref to zero
  virtual bool reset();

private:
  std::string task_name_;
  std::string contact_name_;

  std::atomic<double> weight_{1.0};
  Eigen::Vector3d f_ref_{Eigen::Vector3d::Zero()};
};

} // namespace wolf_wbid
