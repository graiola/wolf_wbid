#include <wolf_wbid/wbid/qp/qp_stack.h>
#include <wolf_wbid/wbid/qp/qp_builder_lsq.h>
#include <wolf_wbid/wbid/id_variables.h>

#include <stdexcept>

namespace wolf_wbid {

QPStack::QPStack(std::string name)
: name_(std::move(name))
{}

void QPStack::clearLevels()
{
  levels_.clear();
}

void QPStack::addLevel(const std::string& level_name, const std::vector<ITaskLSQ*>& tasks)
{
  QPStackLevel lvl;
  lvl.name = level_name;
  lvl.tasks = tasks;
  levels_.push_back(std::move(lvl));
}

void QPStack::setSingleLevel(const std::vector<ITaskLSQ*>& tasks)
{
  levels_.clear();
  addLevel("level0", tasks);
}

void QPStack::setBounds(const Eigen::VectorXd& l, const Eigen::VectorXd& u)
{
  if(l.size() != u.size()) throw std::runtime_error("QPStack: bounds l/u size mismatch");
  l_ = l; u_ = u;
  has_bounds_ = true;
}

void QPStack::clearBounds()
{
  has_bounds_ = false;
  l_.resize(0); u_.resize(0);
}

void QPStack::setLinearConstraints(const Eigen::MatrixXd& A,
                                   const Eigen::VectorXd& lA,
                                   const Eigen::VectorXd& uA)
{
  if(A.rows() != lA.size() || A.rows() != uA.size())
    throw std::runtime_error("QPStack: A/lA/uA mismatch");
  A_ = A; lA_ = lA; uA_ = uA;
  has_lin_ = true;
}

void QPStack::clearLinearConstraints()
{
  has_lin_ = false;
  A_.resize(0,0); lA_.resize(0); uA_.resize(0);
}

void QPStack::setDiagonalRegularization(double reg_eps)
{
  if(reg_eps < 0.0) throw std::runtime_error("QPStack: reg_eps must be >= 0");
  reg_eps_ = reg_eps;
}

void QPStack::update(const IDVariables& vars)
{
  // Merge all tasks from all levels (soft priorities for now)
  std::vector<ITaskLSQ*> all;
  for(auto& lvl : levels_){
    for(auto* t : lvl.tasks){
      if(t) all.push_back(t);
    }
  }

  // Build objective
  QPBuilderLSQ::build(vars, all, qp_, reg_eps_);

  // Attach bounds/constraints
  const int n = vars.size();

  if(has_bounds_){
    if(l_.size() != n || u_.size() != n)
      throw std::runtime_error("QPStack: bounds dimension != vars.size()");
    qp_.l = l_;
    qp_.u = u_;
  } else {
    qp_.l.resize(0);
    qp_.u.resize(0);
  }

  if(has_lin_){
    if(A_.cols() != n)
      throw std::runtime_error("QPStack: A cols != vars.size()");
    qp_.A = A_;
    qp_.lA = lA_;
    qp_.uA = uA_;
  } else {
    qp_.A.resize(0,n);
    qp_.lA.resize(0);
    qp_.uA.resize(0);
  }
}

QPSolution QPStack::solve(IQPSolver& solver)
{
  // assumes update() already called
  return solver.solve(qp_);
}

} // namespace wolf_wbid

