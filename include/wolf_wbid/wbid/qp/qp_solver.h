/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#pragma once
#include <memory>
#include <string>
#include <wolf_wbid/wbid/qp/qp_problem.h>

namespace wolf_wbid {

class IQPSolver
{
public:
  virtual ~IQPSolver() = default;

  virtual std::string name() const = 0;
  virtual void setEpsRegularisation(double eps) { (void)eps; }
  virtual QPSolution solve(const QPProblem& qp) = 0;
};

} // namespace wolf_wbid
