#pragma once
#include <wolf_wbid/wbid/tasks/task_lsq.h>
#include <functional>

namespace wolf_wbid {

/**
 * Minimizza || Sx - ref ||_W
 * dove S seleziona un blocco della variabile decisionale x.
 */
class MinimizeVariableTask final : public ITaskLSQ
{
public:
  using RefGetter = std::function<Eigen::VectorXd(void)>;

  // block_offset, block_dim refer to x layout (via IDVariables)
  MinimizeVariableTask(std::string name,
                       int block_offset,
                       int block_dim,
                       double weight = 1.0);

  std::string name() const { return name_; }

  void setWeight(double w);
  void setReference(const Eigen::VectorXd& ref);   // fixed ref
  void setReferenceGetter(RefGetter g);            // dynamic ref per update()

  void compute(const IDVariables& vars, LsqTerm& out) override;

private:
  std::string name_;
  int offset_{0};
  int dim_{0};
  double weight_{1.0};

  Eigen::VectorXd ref_fixed_;
  RefGetter ref_getter_{nullptr};
};

} // namespace wolf_wbid
