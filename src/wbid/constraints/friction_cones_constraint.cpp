#include <wolf_wbid/wbid/constraints/friction_cones_constraint.h>

#include <stdexcept>

namespace wolf_wbid {

FrictionConesConstraint::FrictionConesConstraint(const std::string& name,
                                                 const std::vector<std::string>& contact_names,
                                                 const IDVariables& vars,
                                                 const std::vector<Eigen::Matrix3d>& wRl_list,
                                                 const std::vector<double>& mu_list)
: ConstraintBase(name)
{
  const std::size_t N = contact_names.size();

  if(wRl_list.size() != N)
    throw std::runtime_error("FrictionConesConstraint: wRl_list size mismatch");
  if(mu_list.size() != N)
    throw std::runtime_error("FrictionConesConstraint: mu_list size mismatch");

  cones_.reserve(N);
  for(std::size_t i = 0; i < N; ++i)
  {
    // ctor: (contact_name, vars, mu, wRl)
    cones_.push_back(std::make_shared<FrictionConeConstraint>(contact_names[i], vars, mu_list[i], wRl_list[i]));
  }

  rebuild();
}

void FrictionConesConstraint::update(const Eigen::VectorXd& /*x*/)
{
  // Se mu / wRl cambiano runtime nei singoli coni:
  // rebuild();
  // Altrimenti, no-op.
}

FrictionConeConstraint::Ptr FrictionConesConstraint::getFrictionCone(const std::string& contact_name)
{
  for(auto& c : cones_)
  {
    if(c && c->contactName() == contact_name)
      return c;
  }
  return nullptr;
}

void FrictionConesConstraint::rebuild()
{
  if(cones_.empty())
  {
    resizeLinear(0, 0);
    return;
  }

  const int nvars = cones_.front()->cols(); // oppure vars.size() se lo passi/stori
  const int m_per = 5;
  const int m_tot = static_cast<int>(cones_.size()) * m_per;

  resizeLinear(m_tot, nvars);

  int r0 = 0;
  for(const auto& c : cones_)
  {
    if(!c) continue;

    // opzionale: rispetta enabled dei singoli coni
    // (se vuoi che un cono "spento" non contribuisca,
    // devi fare rebuild più complesso con righe variabili)
    // Qui assumiamo sempre tutti presenti.

    if(c->A().cols() != nvars)
      throw std::runtime_error("FrictionConesConstraint: cone cols mismatch");

    A_.block(r0, 0, m_per, nvars)  = c->A();
    lA_.segment(r0, m_per)        = c->lA();
    uA_.segment(r0, m_per)        = c->uA();

    r0 += m_per;
  }
}

} // namespace wolf_wbid
