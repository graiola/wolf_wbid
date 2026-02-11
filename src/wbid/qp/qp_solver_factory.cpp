#include <wolf_wbid/wbid/qp/qp_solver_factory.h>
#include <wolf_wbid/wbid/qp/qpoases_solver.h>
#if WOLF_WBID_HAS_EIQUADPROG
#include <wolf_wbid/wbid/qp/eiquadprog_solver.h>
#endif

namespace wolf_wbid {
std::unique_ptr<IQPSolver> CreateDefaultSolver()
{
  return std::make_unique<QPOasesSolver>();
}

std::unique_ptr<IQPSolver> CreateSolverByName(const std::string& name)
{
  if(name == "qpOASES" || name == "qpoases") {
    return std::make_unique<QPOasesSolver>();
  }
#if WOLF_WBID_HAS_EIQUADPROG
  if(name == "eiQuadProg" || name == "eiquadprog") {
    return std::make_unique<EiQuadProgSolver>();
  }
#endif
  return nullptr;
}
}
