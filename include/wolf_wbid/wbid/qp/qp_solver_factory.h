// include/wolf_wbid/wbid/qp/qp_solver_factory.h
#pragma once
#include <memory>
#include <wolf_wbid/wbid/qp/qp_solver.h>

namespace wolf_wbid {
std::unique_ptr<IQPSolver> CreateDefaultSolver();
std::unique_ptr<IQPSolver> CreateSolverByName(const std::string& name);
}
