#include <wolf_wbid/wbid/qp/qp_solver_factory.h>
#include <wolf_wbid/wbid/qp/eiquadprog_solver.h>
#include <wolf_wbid/wbid/qp/qpoases_solver.h>

namespace wolf_wbid {
std::unique_ptr<IQPSolver> CreateDefaultSolver()
{
  return std::make_unique<EiQuadProgSolver>();
}

std::unique_ptr<IQPSolver> CreateSolverByName(const std::string& name)
{
  if(name == "eiQuadProg" || name == "eiquadprog") {
    return std::make_unique<EiQuadProgSolver>();
  }
  if(name == "qpOASES" || name == "qpoases") {
    return std::make_unique<QPOasesSolver>();
  }
  return nullptr;
}
}
