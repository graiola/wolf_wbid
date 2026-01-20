#include <wolf_wbid/id_problem.h>

#include <wolf_wbid/wbid/qp/qp_solver_factory.h>

#include <stdexcept>
#include <limits>

// wrappers ROS
#ifdef ROS
  #include <wolf_wbid/task_ros_wrappers/cartesian.h>
  #include <wolf_wbid/task_ros_wrappers/com.h>
  #include <wolf_wbid/task_ros_wrappers/momentum.h>
  #include <wolf_wbid/task_ros_wrappers/postural.h>
  #include <wolf_wbid/task_ros_wrappers/wrench.h>
#elif defined(ROS2)
  #include <wolf_wbid/task_ros2_wrappers/cartesian.h>
  #include <wolf_wbid/task_ros2_wrappers/com.h>
  #include <wolf_wbid/task_ros2_wrappers/momentum.h>
  #include <wolf_wbid/task_ros2_wrappers/postural.h>
  #include <wolf_wbid/task_ros2_wrappers/wrench.h>
#endif

namespace wolf_wbid {

static inline double kBig() { return 1.0e20; }

IDProblem::IDProblem(QuadrupedRobot::Ptr model)
: model_(std::move(model))
{
  if(!model_) throw std::runtime_error("IDProblem: null model");

  foot_names_    = model_->getFootNames();
  ee_names_      = model_->getEndEffectorNames();
  contact_names_ = model_->getContactNames();

  z_force_lower_lim_ = 0.1 * GRAVITY * (model_->getMass() / N_LEGS);

  wrench_upper_lims_ << 2000, 2000, 2000, 0, 0, 0;
  wrench_lower_lims_ << x_force_lower_lim_, y_force_lower_lim_, z_force_lower_lim_, 0, 0, 0;

  for(const auto& c : foot_names_) contact_enabled_[c] = true;
}

IDProblem::~IDProblem() = default;

void IDProblem::init(const std::string& robot_name, const double& dt)
{
  if(initialized_) return;

  // variables: qddot + point-contact forces
  vars_ = std::make_unique<IDVariables>(model_->getJointNum(),
                                       foot_names_,
                                       IDVariables::ContactModel::POINT_CONTACT);

  const std::string base_link = model_->getBaseLinkName();

  // --- tasks ---------------------------------------------------------------
  // Feet Cartesian tasks
  for(const auto& fn : foot_names_)
  {
    feet_[fn] = std::make_shared<CartesianImpl>(
      robot_name,
      fn,          // task_id
      *model_,
      fn,          // distal link
      base_link,   // base link (NOT world)
      *vars_,
      dt,
      /*use_mesh=*/false
    );

    feet_[fn]->setLambda(0., 0.);
    feet_[fn]->loadParams();
    feet_[fn]->registerReconfigurableVariables();
  }

  // Arms Cartesian tasks
  for(const auto& ee : ee_names_)
  {
    arms_[ee] = std::make_shared<CartesianImpl>(
      robot_name,
      ee,          // task_id
      *model_,
      ee,          // distal link
      base_link,
      *vars_,
      dt,
      /*use_mesh=*/false
    );

    arms_[ee]->setLambda(1., 1.);
    arms_[ee]->OPTIONS.set_ext_reference = true;
    arms_[ee]->loadParams();
    arms_[ee]->registerReconfigurableVariables();
  }

  // Wrench tasks (POINT_CONTACT force tracking)
  for(const auto& fn : foot_names_)
  {
    wrenches_[fn] = std::make_shared<WrenchImpl>(
      robot_name,
      fn + std::string("_wrench"), // task_id
      fn,                          // contact_name
      *vars_,
      dt
    );

    // NOTE: OpenSoT-free wrench has no lambda (so no set_ext_lambda / setLambda)
    // Keep reference/weight configurable if you want:
    wrenches_[fn]->OPTIONS.set_ext_reference = true;
    wrenches_[fn]->loadParams();
    wrenches_[fn]->registerReconfigurableVariables();
  }

  // Angular momentum
  angular_momentum_ = std::make_shared<AngularMomentumImpl>(
    robot_name,
    *model_,
    *vars_,
    "angular_momentum",
    dt
  );

  angular_momentum_->setLambda(0.);
  angular_momentum_->setReference(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  angular_momentum_->setMomentumGain(Eigen::Matrix3d::Identity());
  angular_momentum_->loadParams();
  angular_momentum_->registerReconfigurableVariables();

  // Waist Cartesian task
  waist_ = std::make_shared<CartesianImpl>(
    robot_name,
    "waist",
    *model_,
    base_link,     // distal link = base itself (ok se vuoi controllare la posa del base)
    base_link,     // base link
    *vars_,
    dt,
    /*use_mesh=*/false
  );

  waist_->setLambda(1., 1.);
  waist_->loadParams();
  waist_->registerReconfigurableVariables();

  // Postural
  postural_ = std::make_shared<PosturalImpl>(
    robot_name,
    "postural",
    *model_,
    *vars_,
    dt
  );

  postural_->setLambda(1., 1.);
  postural_->loadParams();
  postural_->registerReconfigurableVariables();
  postural_->setReference(model_->getStandUpJointPostion());

  // CoM
  com_ = std::make_shared<ComImpl>(
    robot_name,
    "com",
    *model_,
    *vars_,
    dt
  );

  com_->setLambda(1., 1.);
  com_->loadParams();
  com_->registerReconfigurableVariables();

  // --- constraints ---------------------------------------------------------
  constraints_.clear();

  // friction cones per foot
  friction_cones_.clear();
  for(const auto& fn : foot_names_)
  {
    // ctor: (contact_name, idvars, mu, R)
    auto fc = std::make_shared<FrictionConeConstraint>(fn, *vars_, mu_, fc_R_);
    friction_cones_[fn] = fc;
    constraints_.push_back(fc);
  }

  // force bounds per foot
  force_bounds_.clear();
  for(const auto& fn : foot_names_)
  {
    Eigen::Vector3d fmin(x_force_lower_lim_, y_force_lower_lim_, z_force_lower_lim_);
    Eigen::Vector3d fmax(wrench_upper_lims_.head<3>());

    auto cb = std::make_shared<ContactForceBoundsConstraint>(fn, *vars_, fmin, fmax);
    force_bounds_[fn] = cb;
    constraints_.push_back(cb);
  }

  // torque limits
  {
    Eigen::VectorXd tau_max;
    model_->getEffortLimits(tau_max);

    // if your model includes floating base dofs in tau_max, keep this:
    if(tau_max.size() >= FLOATING_BASE_DOFS)
      tau_max.head(FLOATING_BASE_DOFS).setZero();

    tau_max = 0.9 * tau_max;

    // ctor: (name, robot, idvars, joints, tau_max)
    torque_limits_ = std::make_shared<TorqueLimitsConstraint>(
      "torque_limits",
      *model_,
      *vars_,
      model_->getJointNames(),   // IMPORTANT: not foot_names_ !
      tau_max
    );

    constraints_.push_back(torque_limits_);
  }

  // solver
  // TODO: replace with your real factory; for now keep what you had
  solver_ = CreateDefaultSolver();
  if(!solver_) throw std::runtime_error("IDProblem::init(): solver factory returned null");

  // buffers
  x_.setZero(vars_->size());
  qddot_.setZero(model_->getJointNum());
  contact_wrenches_.resize(foot_names_.size(), Eigen::Vector6d::Zero());

  initialized_ = true;
}


void IDProblem::setControlMode(mode_t mode)
{
  // currently ignored: single-stack WPG
  if(control_mode_ != mode) {
    control_mode_ = mode;
    change_control_mode_ = true;
  } else {
    change_control_mode_ = false;
  }
}

void IDProblem::setFrictionConesMu(const double& mu)
{
  if(mu < 0.0 || mu > 1.0) throw std::runtime_error("mu out of [0,1]");
  mu_ = mu;
}

double IDProblem::getFrictionConesMu() const { return mu_; }
void IDProblem::setFrictionConesR(const Eigen::Matrix3d& R) { fc_R_ = R; }

void IDProblem::setLowerForceBound(const double& x_force, const double& y_force, const double& z_force)
{
  x_force_lower_lim_ = x_force;
  y_force_lower_lim_ = y_force;
  z_force_lower_lim_ = z_force;
}
void IDProblem::setLowerForceBoundX(const double& f) { x_force_lower_lim_ = f; }
void IDProblem::setLowerForceBoundY(const double& f) { y_force_lower_lim_ = f; }
void IDProblem::setLowerForceBoundZ(const double& f) { z_force_lower_lim_ = f; }

void IDProblem::activateComZ(bool active) { activate_com_z_ = active; }
void IDProblem::activateAngularMomentum(bool active) { activate_angular_momentum_ = active; }
void IDProblem::activatePostural(bool active) { activate_postural_ = active; }
void IDProblem::activateJointPositionLimits(bool active) { activate_joint_position_limits_ = active; }

void IDProblem::setRegularization(double r)
{
  if(r <= 0.0 || r >= 1.0) return;
  regularization_value_ = r;
}

void IDProblem::setForcesMinimizationWeight(double w) { if(w>=0.0) min_forces_weight_ = w; }
void IDProblem::setJointAccelerationMinimizationWeight(double w) { if(w>=0.0) min_qddot_weight_ = w; }

const std::map<std::string,Cartesian::Ptr>& IDProblem::getFootTasks() const { return feet_; }
const std::map<std::string,Cartesian::Ptr>& IDProblem::getArmTasks() const { return arms_; }
const std::map<std::string,Wrench::Ptr>& IDProblem::getWrenchTasks() const { return wrenches_; }
const Cartesian::Ptr& IDProblem::getWaistTask() const { return waist_; }
const Com::Ptr& IDProblem::getComTask() const { return com_; }

void IDProblem::activateExternalReferences(bool activate)
{
  for(const auto& fn : foot_names_) feet_[fn]->OPTIONS.set_ext_reference = activate;
  for(const auto& fn : foot_names_) wrenches_[fn]->OPTIONS.set_ext_reference = activate;
  waist_->OPTIONS.set_ext_reference = activate;
  postural_->OPTIONS.set_ext_reference = activate;
  com_->OPTIONS.set_ext_reference = activate;
}

void IDProblem::reset()
{
  for(auto& kv : arms_)     { kv.second->update(Eigen::VectorXd(1)); kv.second->reset(); }
  for(auto& kv : feet_)     { kv.second->update(Eigen::VectorXd(1)); kv.second->reset(); }
  for(auto& kv : wrenches_) { kv.second->update(Eigen::VectorXd(1)); kv.second->reset(); }
  waist_->update(Eigen::VectorXd(1)); waist_->reset();
  com_->update(Eigen::VectorXd(1));   com_->reset();

  change_control_mode_ = false;
}

void IDProblem::setFootReference(const std::string& foot_name,
                                 const Eigen::Affine3d& pose_ref,
                                 const Eigen::Vector6d& vel_ref,
                                 const std::string& reference_frame)
{
  if(reference_frame == model_->getBaseLinkName()) {
    feet_.at(foot_name)->setReference(pose_ref, vel_ref);
    return;
  }
  if(reference_frame == WORLD_FRAME_NAME) {
    tmp_affine3d_ = model_->getBasePoseInWorld().inverse(); // base_T_world
    tmp_vector6d_.setZero();
    tmp_vector6d_.head<3>() = tmp_affine3d_.linear() * vel_ref.head<3>();
    tmp_affine3d_ = tmp_affine3d_ * pose_ref;
    feet_.at(foot_name)->setReference(tmp_affine3d_, tmp_vector6d_);
    return;
  }
  throw std::runtime_error("Wrong reference frame in setFootReference()");
}

void IDProblem::setWaistReference(const Eigen::Matrix3d& Rot, const double& z, const double& z_vel)
{
  tmp_vector6d_.setZero();
  tmp_affine3d_.setIdentity();
  tmp_affine3d_.linear() = Rot;
  tmp_affine3d_.translation().z() = z;
  tmp_vector6d_(2) = z_vel;
  waist_->setReference(tmp_affine3d_, tmp_vector6d_);
}

void IDProblem::setComReference(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity)
{
  com_->setReference(position, velocity);
}

void IDProblem::setPosture(const Eigen::MatrixXd& Kp, const Eigen::MatrixXd& Kd, const Eigen::VectorXd& q)
{
  postural_->setGains(Kp, Kd);
  postural_->setReference(q);
}

void IDProblem::swingWithFoot(const std::string& foot_name, const std::string& ref_frame)
{
  feet_.at(foot_name)->setBaseLink(ref_frame);
  feet_.at(foot_name)->setLambda(1., 1.);

  contact_enabled_[foot_name] = false;

  if(force_bounds_.count(foot_name)) force_bounds_.at(foot_name)->releaseContact(true);
  if(torque_limits_) torque_limits_->disableContact(foot_name);
}

void IDProblem::stanceWithFoot(const std::string& foot_name, const std::string& ref_frame)
{
  feet_.at(foot_name)->setBaseLink(ref_frame);
  feet_.at(foot_name)->setLambda(0., 0.);

  contact_enabled_[foot_name] = true;

  if(force_bounds_.count(foot_name)) force_bounds_.at(foot_name)->releaseContact(false);
  if(torque_limits_) torque_limits_->enableContact(foot_name);
}

void IDProblem::publish()
{
  for(auto& kv : feet_) kv.second->publish();
  for(auto& kv : arms_) kv.second->publish();
  waist_->publish();
  com_->publish();
  postural_->publish();
  angular_momentum_->publish();
}

void IDProblem::update()
{
  // Update limits
  wrench_lower_lims_(0) = x_force_lower_lim_;
  wrench_lower_lims_(1) = y_force_lower_lim_;
  wrench_lower_lims_(2) = z_force_lower_lim_;

  // Update constraints params
  for(const auto& fn : foot_names_)
  {
    if(friction_cones_.count(fn))
    {
      friction_cones_.at(fn)->setContactRotation(fc_R_);
      friction_cones_.at(fn)->setMu(mu_);
      friction_cones_.at(fn)->setEnabled(contact_enabled_[fn]); // optional policy: disable on swing
    }

    if(force_bounds_.count(fn))
    {
      Eigen::Vector3d fmin(x_force_lower_lim_, y_force_lower_lim_, z_force_lower_lim_);
      Eigen::Vector3d fmax(wrench_upper_lims_.head<3>());
      force_bounds_.at(fn)->setMin(fmin);
      force_bounds_.at(fn)->setMax(fmax);
      force_bounds_.at(fn)->releaseContact(!contact_enabled_[fn]); // release on swing
      force_bounds_.at(fn)->setEnabled(true); // keep enabled; releaseContact controls bounds
    }
  }

  // Tasks update with dummy x
  Eigen::VectorXd x_dummy = Eigen::VectorXd::Zero(vars_->size());

  for(auto& kv : feet_)     kv.second->update(x_dummy);
  for(auto& kv : arms_)     kv.second->update(x_dummy);
  for(auto& kv : wrenches_) kv.second->update(x_dummy);
  waist_->update(x_dummy);
  com_->update(x_dummy);
  postural_->update(x_dummy);
  angular_momentum_->update(x_dummy);
}

void IDProblem::setDefaultBounds(QPProblem& qp)
{
  qp.l.setConstant(qp.n(), -kBig());
  qp.u.setConstant(qp.n(),  kBig());
}

void IDProblem::addLeastSquaresTerm(QPProblem& qp,
                                   const Eigen::MatrixXd& A,
                                   const Eigen::VectorXd& b,
                                   const Eigen::MatrixXd& W)
{
  qp.H.noalias() += A.transpose() * W * A;
  qp.g.noalias() += -A.transpose() * W * b;
}

void IDProblem::addLeastSquaresRows(QPProblem& qp,
                                   const Eigen::MatrixXd& A,
                                   const Eigen::VectorXd& b,
                                   const Eigen::MatrixXd& W,
                                   const std::vector<int>& rows)
{
  const int r = static_cast<int>(rows.size());
  if(r <= 0) return;

  Eigen::MatrixXd Ar(r, A.cols());
  Eigen::VectorXd br(r);
  Eigen::MatrixXd Wr(r, r);
  Ar.setZero();
  br.setZero();
  Wr.setZero();

  for(int i = 0; i < r; ++i)
  {
    const int ri = rows[i];
    Ar.row(i) = A.row(ri);
    br(i) = b(ri);
    for(int j = 0; j < r; ++j) {
      Wr(i,j) = W(ri, rows[j]);
    }
  }

  addLeastSquaresTerm(qp, Ar, br, Wr);
}

void IDProblem::applyConstraintContributions(QPProblem& qp,
                                            const std::vector<std::shared_ptr<IConstraint>>& constraints)
{
  // 1) merge bounds contributions (ConstraintBase only)
  for(const auto& c : constraints)
  {
    if(!c) continue;
    if(!c->enabled()) continue;

    const auto* cb = dynamic_cast<const ConstraintBase*>(c.get());
    if(cb && cb->hasBounds())
    {
      // IMPORTANT: bounds vectors are full-size (nvars) by your design
      if(cb->l().size() != qp.n() || cb->u().size() != qp.n())
        throw std::runtime_error("Constraint bounds size mismatch in " + c->name());

      qp.l = qp.l.cwiseMax(cb->l());
      qp.u = qp.u.cwiseMin(cb->u());
    }
  }

  // 2) stack linear constraints A/lA/uA
  int m_total = 0;
  for(const auto& c : constraints)
  {
    if(!c) continue;
    if(!c->enabled()) continue;
    m_total += c->rows();
  }

  qp.A.resize(m_total, qp.n());
  qp.lA.resize(m_total);
  qp.uA.resize(m_total);

  int row = 0;
  for(const auto& c : constraints)
  {
    if(!c) continue;
    if(!c->enabled()) continue;

    const int mi = c->rows();
    if(mi <= 0) continue;

    if(c->cols() != qp.n())
      throw std::runtime_error("Constraint cols mismatch in " + c->name());

    if(c->A().rows() != mi || c->A().cols() != qp.n())
      throw std::runtime_error("Constraint A size mismatch in " + c->name());

    qp.A.block(row, 0, mi, qp.n()) = c->A();
    qp.lA.segment(row, mi) = c->lA();
    qp.uA.segment(row, mi) = c->uA();
    row += mi;
  }
}

bool IDProblem::buildQP(QPProblem& qp)
{
  qp.resize(vars_->size(), 0);
  setDefaultBounds(qp);

  qp.H.setZero();
  qp.g.setZero();

  // regularization
  qp.H.diagonal().array() += regularization_value_;

  // block diagonal minimization weights (optional)
  if(min_qddot_weight_ > 0.0)
  {
    const auto& b = vars_->qddotBlock();
    qp.H.block(b.offset, b.offset, b.dim, b.dim).diagonal().array() += min_qddot_weight_;
  }

  if(min_forces_weight_ > 0.0)
  {
    for(const auto& fn : foot_names_)
    {
      const auto& b = vars_->contactBlock(fn);
      qp.H.block(b.offset, b.offset, b.dim, b.dim).diagonal().array() += min_forces_weight_;
    }
  }

  // --- tasks -> LS
  // feet XYZ only
  for(const auto& fn : foot_names_)
  {
    const auto& t = *feet_.at(fn);
    addLeastSquaresRows(qp, t.A(), t.b(), t.W(), {0,1,2});
  }

  // arms full 6D
  for(const auto& ee : ee_names_)
  {
    const auto& t = *arms_.at(ee);
    addLeastSquaresTerm(qp, t.A(), t.b(), t.W());
  }

  // waist + com split
  {
    const auto& w = *waist_;
    const auto& c = *com_;

    if(activate_com_z_)
    {
      addLeastSquaresRows(qp, w.A(), w.b(), w.W(), {3,4,5});
      addLeastSquaresRows(qp, c.A(), c.b(), c.W(), {0,1,2});
    }
    else
    {
      addLeastSquaresRows(qp, w.A(), w.b(), w.W(), {2,3,4,5});
      addLeastSquaresRows(qp, c.A(), c.b(), c.W(), {0,1});
    }
  }

  if(activate_angular_momentum_)
  {
    const auto& t = *angular_momentum_;
    addLeastSquaresTerm(qp, t.A(), t.b(), t.W());
  }

  if(activate_postural_)
  {
    const auto& t = *postural_;
    addLeastSquaresTerm(qp, t.A(), t.b(), t.W());
  }

  // --- constraints update + merge
  for(const auto& c : constraints_)
  {
    if(!c) continue;
    if(!c->enabled()) continue;
    c->update(x_); // can ignore x internally; safe to call
  }

  applyConstraintContributions(qp, constraints_);

  // NOTE: joint position limits not yet implemented here (activate_joint_position_limits_)

  return true;
}

bool IDProblem::solveQP(IQPSolver& solver, QPProblem& qp, Eigen::VectorXd& x)
{
  return solver.solve(qp, x);
}

bool IDProblem::computeTauFromSolution(const Eigen::VectorXd& x, Eigen::VectorXd& tau)
{
  return vars_->computeTorque(*model_, x, tau, qddot_, contact_wrenches_);
}

bool IDProblem::solve(Eigen::VectorXd& tau)
{
  if(!initialized_) return false;

  update();

  if(!buildQP(qp_)) return false;
  if(!solver_) return false;

  if(!solveQP(*solver_, qp_, x_)) return false;

  return computeTauFromSolution(x_, tau);
}

const std::vector<Eigen::Vector6d>& IDProblem::getContactWrenches() const { return contact_wrenches_; }
const Eigen::VectorXd& IDProblem::getJointAccelerations() const { return qddot_; }

} // namespace wolf_wbid
