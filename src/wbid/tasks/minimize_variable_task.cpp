#include <wolf_wbid/wbid/tasks/minimize_variable_task.h>
#include <wolf_wbid/wbid/id_variables.h>
#include <stdexcept>

namespace wolf_wbid {

MinimizeVariableTask::MinimizeVariableTask(std::string name,
                                           int block_offset,
                                           int block_dim,
                                           double weight)
: name_(std::move(name)), offset_(block_offset), dim_(block_dim), weight_(weight)
{
  if(dim_ <= 0) throw std::runtime_error("MinimizeVariableTask: dim <= 0");
  if(weight_ < 0.0) throw std::runtime_error("MinimizeVariableTask: negative weight");
  ref_fixed_.setZero(dim_);
}

void MinimizeVariableTask::setWeight(double w)
{
  if(w < 0.0) throw std::runtime_error("MinimizeVariableTask: negative weight");
  weight_ = w;
}

void MinimizeVariableTask::setReference(const Eigen::VectorXd& ref)
{
  if(ref.size() != dim_) throw std::runtime_error("MinimizeVariableTask: ref size mismatch");
  ref_fixed_ = ref;
  ref_getter_ = nullptr;
}

void MinimizeVariableTask::setReferenceGetter(RefGetter g)
{
  ref_getter_ = std::move(g);
}

void MinimizeVariableTask::compute(const IDVariables& vars, LsqTerm& out)
{
  const int n = vars.size();
  if(offset_ + dim_ > n) throw std::runtime_error("MinimizeVariableTask: block out of range");

  out.A.setZero(dim_, n);
  out.b.setZero(dim_);
  out.w_diag.setOnes(dim_);

  // selector
  out.A.block(0, offset_, dim_, dim_) = Eigen::MatrixXd::Identity(dim_, dim_);

  // reference
  if(ref_getter_){
    Eigen::VectorXd r = ref_getter_();
    if(r.size() != dim_) throw std::runtime_error("MinimizeVariableTask: dynamic ref size mismatch");
    out.b = r;
  } else {
    out.b = ref_fixed_;
  }

  out.w_diag.setConstant(weight_);
}

} // namespace wolf_wbid
