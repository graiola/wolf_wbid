#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace wolf_wbid {

class IDVariables;

struct BoundsBuilder
{
  static void initInf(const IDVariables& vars, Eigen::VectorXd& l, Eigen::VectorXd& u,
                      double inf = 1e20);

  static void setBlockBounds(Eigen::VectorXd& l, Eigen::VectorXd& u,
                             int offset, int dim,
                             const Eigen::VectorXd& l_block,
                             const Eigen::VectorXd& u_block);

  static void setContactForceBoundsZ(Eigen::VectorXd& l, Eigen::VectorXd& u,
                                     const IDVariables& vars,
                                     const std::string& contact_name,
                                     double fz_min,
                                     double fz_max);
};

} // namespace wolf_wbid

