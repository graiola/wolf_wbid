#include <wolf_wbid/wbid/tasks/aggregated_task_lsq.h>
#include <wolf_wbid/wbid/id_variables.h>
#include <stdexcept>

namespace wolf_wbid {

void AggregatedTaskLSQ::compute(const IDVariables& vars, LsqTerm& out)
{
  // First pass: compute sizes
  int total_m = 0;
  int n = vars.size();

  LsqTerm tmp;
  std::vector<LsqTerm> terms;
  terms.reserve(tasks_.size());

  for(auto* t : tasks_){
    if(!t) continue;
    t->compute(vars, tmp);
    if(tmp.A.cols() != n) throw std::runtime_error("AggregatedTaskLSQ: A cols != vars.size()");
    if(tmp.A.rows() != tmp.b.size() || tmp.b.size() != tmp.w_diag.size())
      throw std::runtime_error("AggregatedTaskLSQ: term dims mismatch");
    total_m += static_cast<int>(tmp.b.size());
    terms.push_back(tmp);
  }

  out.A.setZero(total_m, n);
  out.b.setZero(total_m);
  out.w_diag.setOnes(total_m);

  int off = 0;
  for(const auto& t : terms){
    const int m = static_cast<int>(t.b.size());
    out.A.block(off, 0, m, n) = t.A;
    out.b.segment(off, m) = t.b;
    out.w_diag.segment(off, m) = t.w_diag;
    off += m;
  }
}

} // namespace wolf_wbid

