#include <wolf_wbid/wbid/qp/bounds_builder.h>
#include <wolf_wbid/wbid/id_variables.h>
#include <stdexcept>

namespace wolf_wbid {

void BoundsBuilder::initInf(const IDVariables& vars, Eigen::VectorXd& l, Eigen::VectorXd& u, double inf)
{
  const int n = vars.size();
  l = Eigen::VectorXd::Constant(n, -inf);
  u = Eigen::VectorXd::Constant(n,  inf);
}

void BoundsBuilder::setBlockBounds(Eigen::VectorXd& l, Eigen::VectorXd& u,
                                   int offset, int dim,
                                   const Eigen::VectorXd& l_block,
                                   const Eigen::VectorXd& u_block)
{
  if(l.size() != u.size()) throw std::runtime_error("BoundsBuilder: l/u mismatch");
  if(l_block.size() != dim || u_block.size() != dim) throw std::runtime_error("BoundsBuilder: block size mismatch");
  l.segment(offset, dim) = l_block;
  u.segment(offset, dim) = u_block;
}

void BoundsBuilder::setContactForceBoundsZ(Eigen::VectorXd& l, Eigen::VectorXd& u,
                                          const IDVariables& vars,
                                          const std::string& contact_name,
                                          double fz_min, double fz_max)
{
  // assumes POINT_CONTACT => dim 3, z is index 2
  const int off = vars.contactOffset(contact_name);
  const int cd  = vars.contactDim();
  if(cd < 3) throw std::runtime_error("BoundsBuilder: contactDim < 3");

  l(off + 2) = fz_min;
  u(off + 2) = fz_max;
}

} // namespace wolf_wbid

