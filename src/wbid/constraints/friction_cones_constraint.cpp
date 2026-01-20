#include <wolf_wbid/wbid/constraints/friction_cones_constraint.h>

#include <stdexcept>

namespace wolf_wbid {

FrictionConesConstraint::FrictionConesConstraint(const std::string& name,
                                                 const std::vector<std::string>& contact_names,
                                                 const IDVariables& vars,
                                                 const std::vector<Eigen::Matrix3d>& wRl_list,
                                                 const std::vector<double>& mu_list)
  : name_(name)
{
  if(contact_names.empty())
    throw std::invalid_argument("FrictionConesConstraint: empty contact list");

  if(wRl_list.size() != contact_names.size() || mu_list.size() != contact_names.size())
    throw std::invalid_argument("FrictionConesConstraint: size mismatch");

  cones_.reserve(contact_names.size());
  for(size_t i = 0; i < contact_names.size(); ++i)
  {
    cones_.push_back(std::make_shared<FrictionConeConstraint>(
        contact_names[i] + "_friction_cone",
        contact_names[i],
        vars,
        wRl_list[i],
        mu_list[i]));
  }

  rebuild();
}

void FrictionConesConstraint::update(const Eigen::VectorXd& /*x*/)
{
  // no-op
}

FrictionConeConstraint::Ptr FrictionConesConstraint::getFrictionCone(const std::string& contact_name)
{
  for(auto& c : cones_)
    if(c->contactName() == contact_name) return c;
  return nullptr;
}

void FrictionConesConstraint::rebuild()
{
  // Stack all cones:
  // each one is 5 x nvars
  const int nvars = (cones_.empty() ? 0 : static_cast<int>(cones_[0]->Aineq().cols()));
  const int rows = 5 * static_cast<int>(cones_.size());

  Aineq_.setZero(rows, nvars);
  bUpper_.setZero(rows);
  bLower_.setZero(rows);

  for(int i = 0; i < static_cast<int>(cones_.size()); ++i)
  {
    const int r0 = 5 * i;
    Aineq_.block(r0, 0, 5, nvars) = cones_[i]->Aineq();
    bUpper_.segment(r0, 5) = cones_[i]->bUpperBound();
    bLower_.segment(r0, 5) = cones_[i]->bLowerBound();
  }
}

} // namespace wolf_wbid

