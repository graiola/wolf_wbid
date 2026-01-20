#pragma once
#include <wolf_wbid/wbid/task_lsq.h>
#include <vector>

namespace wolf_wbid {

class AggregatedTaskLSQ final : public ITaskLSQ
{
public:
  AggregatedTaskLSQ(std::string name, std::vector<ITaskLSQ*> tasks)
  : name_(std::move(name)), tasks_(std::move(tasks)) {}

  std::string name() const override { return name_; }

  void compute(const IDVariables& vars, LsqTerm& out) override;

private:
  std::string name_;
  std::vector<ITaskLSQ*> tasks_; // non-owning
};

} // namespace wolf_wbid

