/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#pragma once
#include <wolf_wbid/wbid/qp/qp_solver.h>

namespace wolf_wbid {

class EiQuadProgSolver final : public IQPSolver
{
public:
  EiQuadProgSolver();

  std::string name() const override { return "eiQuadProg"; }

  void setEpsRegularisation(double eps) override;

  QPSolution solve(const QPProblem& qp) override;

private:

  static constexpr double BASE_REGULARISATION = 2.22E-13;

  double eps_reg_{BASE_REGULARISATION}; // already scaled
};

} // namespace wolf_wbid
