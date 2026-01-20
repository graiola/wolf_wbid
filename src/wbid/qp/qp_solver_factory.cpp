#include <wolf_wbid/wbid/qp/qp_solver_factory.h>
#include <wolf_wbid/wbid/qp/eiquadprog_solver.h>

namespace wolf_wbid {
std::unique_ptr<IQPSolver> CreateDefaultSolver()
{
  return std::make_unique<EiQuadProgSolver>();
}
}

