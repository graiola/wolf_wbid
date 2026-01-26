#include <wolf_wbid/id_problem.h>

#include <wolf_wbid/wbid/qp/qp_solver_factory.h>

#include <stdexcept>
#include <limits>
#include <algorithm>
#include <cmath>

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

static inline std::vector<int> rowsXYZ() { return {0,1,2}; }
static inline std::vector<int> rowsXY()  { return {0,1}; }
static inline std::vector<int> rowsZ()   { return {2}; }
static inline std::vector<int> rowsRPY() { return {3,4,5}; }

static inline std::vector<int> rowsLimbs(int task_size)
{
  std::vector<int> r;
  r.reserve(std::max(0, task_size - FLOATING_BASE_DOFS));
  for(int i = FLOATING_BASE_DOFS; i < task_size; ++i) r.push_back(i);
  return r;
}

// ----------------------------------------------------------------------------
// Block regularization (qddot + eps, contacts + eps^2)
// ----------------------------------------------------------------------------
static inline void applyBlockRegularization(Eigen::MatrixXd& H,
                                            const IDVariables& vars,
                                            const std::vector<std::string>& contacts,
                                            double eps)
{
  if(!(std::isfinite(eps) && eps > 0.0)) {
    return;
  }

  const double eps2 = eps * eps;

  // qddot block
  const auto& qb = vars.qddotBlock();
  if(qb.dim > 0) {
    H.block(qb.offset, qb.offset, qb.dim, qb.dim).diagonal().array() += eps;
  }

  // contact blocks
  for(const auto& cname : contacts)
  {
    if(!vars.hasContact(cname)) continue;
    const auto& cb = vars.contactBlock(cname);
    if(cb.dim > 0) {
      H.block(cb.offset, cb.offset, cb.dim, cb.dim).diagonal().array() += eps2;
    }
  }
}

// ----------------------------------------------------------------------------
// OpenSoT-compat weights patch:
// - if W is (almost) diagonal, square the diagonal -> W_eff = diag(w)^2
// - else keep W unchanged (non-diagonal weights are rare; squaring them changes semantics)
// ----------------------------------------------------------------------------
static inline bool isAlmostDiagonal(const Eigen::MatrixXd& W, double tol = 1e-12)
{
  if(W.rows() != W.cols()) return false;
  const int n = W.rows();
  for(int i=0;i<n;++i){
    for(int j=0;j<n;++j){
      if(i==j) continue;
      if(std::abs(W(i,j)) > tol) return false;
    }
  }
  return true;
}

static inline Eigen::MatrixXd makeWeightCompat_OpenSoT(const Eigen::MatrixXd& W)
{
  if(W.size() == 0) return W;

  // Typical tasks use diagonal weights
  if(isAlmostDiagonal(W)) {
    Eigen::MatrixXd We = W;
    We.diagonal().array() = We.diagonal().array().square();
    return We;
  }

  // Non-diagonal: keep as-is (safe)
  // If you *really* want to force squaring, tell me and we can do:
  //   - We = W * W  (matrix product)  OR
  //   - We = W.cwiseProduct(W) (element-wise)
  return W;
}

IDProblem::IDProblem(QuadrupedRobot::Ptr model)
: model_(std::move(model))
, control_mode_(WPG)
, initialized_(false)
, activate_com_z_(true)
, activate_angular_momentum_(true)
, activate_postural_(false)
, activate_joint_position_limits_(false)
, change_control_mode_(false)
, regularization_value_(1e-3)
, min_forces_weight_(0.0)
, min_qddot_weight_(0.0)
{
  if(!model_) throw std::runtime_error("IDProblem: null model");

  foot_names_    = model_->getFootNames();
  ee_names_      = model_->getEndEffectorNames();
  contact_names_ = model_->getContactNames();

  // defaults (match old behavior)
  x_force_lower_lim_ = -2000.0;
  y_force_lower_lim_ = -2000.0;
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
      fn,                // task_id
      *model_,
      fn,                // distal link
      WORLD_FRAME_NAME,  // base link (world)
      *vars_,
      dt,
      /*use_mesh=*/false
    );
    feet_[fn]->setGainType(CartesianTask::GainType::Force);
    feet_[fn]->setLambda(0., 0.);                  // WPG default
    feet_[fn]->OPTIONS.set_ext_lambda = false;     // WPG default
    feet_[fn]->loadParams();
    feet_[fn]->registerReconfigurableVariables();
  }

  // Arms Cartesian tasks (external refs by default, as in old code note)
  for(const auto& ee : ee_names_)
  {
    arms_[ee] = std::make_shared<CartesianImpl>(
      robot_name,
      ee,        // task_id
      *model_,
      ee,        // distal link
      base_link, // base link
      *vars_,
      dt,
      /*use_mesh=*/false
    );
    arms_[ee]->setGainType(CartesianTask::GainType::Acceleration);
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
    // keep reference/weight configurable
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
    base_link,         // distal link = base itself
    WORLD_FRAME_NAME,  // base link (world)
    *vars_,
    dt,
    /*use_mesh=*/false
  );
  waist_->setGainType(CartesianTask::GainType::Force);
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
  postural_->setGainType(PosturalTask::GainType::Force);
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
    auto fc = std::make_shared<FrictionConeConstraint>(fn, *vars_, mu_, fc_R_);
    friction_cones_[fn] = fc;
    constraints_.push_back(fc);
  }

  // force bounds per foot
  force_bounds_.clear();
  for(const auto& fn : foot_names_)
  {
    Eigen::Vector3d fmin(x_force_lower_lim_, y_force_lower_lim_, z_force_lower_lim_);
    Eigen::Vector3d fmax = wrench_upper_lims_.head<3>();

    auto cb = std::make_shared<ContactForceBoundsConstraint>(fn, *vars_, fmin, fmax);
    force_bounds_[fn] = cb;
    constraints_.push_back(cb);
  }

  // torque limits
  {
    Eigen::VectorXd tau_max;
    model_->getEffortLimits(tau_max);
    if(tau_max.size() >= FLOATING_BASE_DOFS)
      tau_max.head(FLOATING_BASE_DOFS).setZero();
    tau_max = 0.9 * tau_max;

    torque_limits_ = std::make_shared<TorqueLimitsConstraint>(
      "torque_limits",
      *model_,
      *vars_,
      foot_names_,
      tau_max
    );
    constraints_.push_back(torque_limits_);
  }

  // --- hard constraint: prevent "falling solution" (qddot_base_z = 0)
  //base_accel_z_ = std::make_shared<BaseAccelZConstraint>(
  //  "base_accel_z",
  //  *vars_,
  //  BaseAccelZConstraint::Mode::EQUALITY,
  //  /*eps=*/0.0
  //);
  //base_accel_z_->setEnabled(true);
  //constraints_.push_back(base_accel_z_);

  // --- floating base accel constraint (coerente con base_accel_z)
  // Consiglio per stand-up: blocca tutto tranne z:
  //   x,y,roll,pitch,yaw = 0
  // (z lo gestisce base_accel_z_)
  //base_accel_fb_ = std::make_shared<FloatingBaseAccelConstraint>(
  //  "base_accel_fb",
  //  *vars_,
  //  /*base_indices=*/std::vector<int>{2, 3, 4},
  //  FloatingBaseAccelConstraint::Mode::EQUALITY,
  //  /*eps=*/0.5
  //);
  //base_accel_fb_->setEnabled(true);
  //constraints_.push_back(base_accel_fb_);

  // solver
  solver_ = CreateDefaultSolver();
  if(!solver_) throw std::runtime_error("IDProblem::init(): solver factory returned null");

  // buffers
  x_.setZero(vars_->size());
  qddot_.setZero(model_->getJointNum());
  contact_wrenches_.resize(foot_names_.size(), Eigen::Vector6d::Zero());

  // replicate old behavior: start in WPG (no external refs)
  control_mode_ = WPG;
  change_control_mode_ = true;
  update();
  change_control_mode_ = false;

  initialized_ = true;
}

void IDProblem::setControlMode(mode_t mode)
{
  if(mode != WPG && mode != EXT) return;

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
  for(const auto& fn : foot_names_)     feet_[fn]->OPTIONS.set_ext_reference = activate;
  for(const auto& fn : foot_names_)     wrenches_[fn]->OPTIONS.set_ext_reference = activate;
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
    tmp_affine3d_ = model_->getBasePoseInWorld().inverse();
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
  if(dynamics_eq_) dynamics_eq_->disableContact(foot_name);
}

void IDProblem::stanceWithFoot(const std::string& foot_name, const std::string& ref_frame)
{
  feet_.at(foot_name)->setBaseLink(ref_frame);
  feet_.at(foot_name)->setLambda(0., 0.);

  contact_enabled_[foot_name] = true;

  if(force_bounds_.count(foot_name)) force_bounds_.at(foot_name)->releaseContact(false);
  if(torque_limits_) torque_limits_->enableContact(foot_name);
  if(dynamics_eq_) dynamics_eq_->enableContact(foot_name);
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
      friction_cones_.at(fn)->setEnabled(contact_enabled_[fn]);
    }

    if(force_bounds_.count(fn))
    {
      Eigen::Vector3d fmin(x_force_lower_lim_, y_force_lower_lim_, z_force_lower_lim_);
      Eigen::Vector3d fmax = wrench_upper_lims_.head<3>();
      force_bounds_.at(fn)->setMin(fmin);
      force_bounds_.at(fn)->setMax(fmax);
      force_bounds_.at(fn)->releaseContact(!contact_enabled_[fn]);
      force_bounds_.at(fn)->setEnabled(true);
    }
  }

  // Mode switch logic
  if(change_control_mode_)
  {
    if(control_mode_ == EXT)
    {
      for(size_t i = 0; i < foot_names_.size(); ++i)
      {
        const auto& fn = foot_names_[i];

        if(i < contact_wrenches_.size())
          wrenches_[fn]->setReference(contact_wrenches_[i].head<3>());

        feet_[fn]->setLambda(1., 1.);
      }

      waist_->setReference(model_->getBasePoseInWorld());
      model_->getCOM(tmp_vector3d_);
      tmp_vector3d_1_.setZero();
      com_->setReference(tmp_vector3d_, tmp_vector3d_1_);

      reset();
      activateExternalReferences(true);
    }
    else // WPG
    {
      for(size_t i = 0; i < foot_names_.size(); ++i)
      {
        const auto& fn = foot_names_[i];

        if(i < contact_wrenches_.size())
          wrenches_[fn]->setReference(contact_wrenches_[i].head<3>());

        feet_[fn]->setBaseLink(WORLD_FRAME_NAME);
        feet_[fn]->setLambda(0., 0.);
      }

      waist_->setReference(model_->getBasePoseInWorld());
      model_->getCOM(tmp_vector3d_);
      tmp_vector3d_1_.setZero();
      com_->setReference(tmp_vector3d_, tmp_vector3d_1_);

      reset();
      activateExternalReferences(false);
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
  // OpenSoT-compat: square diagonal weights
  const Eigen::MatrixXd We = makeWeightCompat_OpenSoT(W);

  qp.H.noalias() += A.transpose() * We * A;
  qp.g.noalias() += -A.transpose() * We * b;
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

  // Regularization
  applyBlockRegularization(qp.H, *vars_, foot_names_, regularization_value_);

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
  for(const auto& fn : foot_names_)
  {
    const auto& t = *feet_.at(fn);
    addLeastSquaresRows(qp, t.A(), t.b(), t.W(), {0,1,2});
  }

  for(const auto& ee : ee_names_)
  {
    const auto& t = *arms_.at(ee);
    addLeastSquaresTerm(qp, t.A(), t.b(), t.W());
  }

  // waist + com split
  {
    const auto& w = *waist_;
    const auto& c = *com_;

    if(activate_com_z_) {
      addLeastSquaresRows(qp, w.A(), w.b(), w.W(), rowsRPY());
      addLeastSquaresRows(qp, c.A(), c.b(), c.W(), rowsXYZ());
    } else {
      auto wzrpy = rowsRPY();
      wzrpy.insert(wzrpy.begin(), 2);
      addLeastSquaresRows(qp, w.A(), w.b(), w.W(), wzrpy);
      addLeastSquaresRows(qp, c.A(), c.b(), c.W(), rowsXY());
    }
  }

  if(activate_angular_momentum_)
  {
    const auto& t = *angular_momentum_;
    addLeastSquaresTerm(qp, t.A(), t.b(), t.W());
  }

  // if(activate_postural_) { ... }

  // --- constraints update + merge
  for(const auto& c : constraints_)
  {
    if(!c) continue;
    if(!c->enabled()) continue;
    c->update(x_);
  }

  applyConstraintContributions(qp, constraints_);

  return true;
}

bool IDProblem::solveQP(IQPSolver& solver, const QPProblem& qp, Eigen::VectorXd& x)
{
  const QPSolution sol = solver.solve(qp);

  if(!sol.success) {
    return false;
  }

  if(sol.x.size() != qp.n()) {
    throw std::runtime_error("IDProblem::solveQP(): solver returned wrong x size");
  }

  x = sol.x;
  return true;
}

bool IDProblem::computeTauFromSolution(const Eigen::VectorXd& x, Eigen::VectorXd& tau)
{
  return vars_->computeTorque(*model_, x, tau, qddot_, contact_wrenches_);
}

bool IDProblem::solve(Eigen::VectorXd& tau)
{
  if(!initialized_) return false;

  update();

  bool qp_built = buildQP(qp_);
  if(!qp_built) {
    debugDumpSolveStep(&qp_, nullptr, nullptr, false, false, false);
    return false;
  }

  if(!solver_) {
    debugDumpSolveStep(&qp_, nullptr, nullptr, true, false, false);
    return false;
  }

  bool qp_solved = solveQP(*solver_, qp_, x_);
  if(!qp_solved) {
    debugDumpSolveStep(&qp_, &x_, nullptr, true, false, false);
    return false;
  }

  bool torque_ok = computeTauFromSolution(x_, tau);

  debugDumpSolveStep(&qp_, &x_, &tau, true, true, torque_ok);

  return torque_ok;
}

const std::vector<Eigen::Vector6d>& IDProblem::getContactWrenches() const { return contact_wrenches_; }
const Eigen::VectorXd& IDProblem::getJointAccelerations() const { return qddot_; }

// -----------------------------------------------------------------------------
// Debug helpers (unchanged)
// -----------------------------------------------------------------------------

void IDProblem::debugPrintVecStats(const std::string& name, const Eigen::VectorXd& v)
{
  if(v.size() == 0) {
    std::cout << "[DBG] " << name << ": <empty>\n";
    return;
  }
  const double n = v.norm();
  const double minv = v.minCoeff();
  const double maxv = v.maxCoeff();
  std::cout << "[DBG] " << name
            << " size=" << v.size()
            << " norm=" << n
            << " min=" << minv
            << " max=" << maxv
            << "\n";
}

void IDProblem::debugPrintMatStats(const std::string& name, const Eigen::MatrixXd& M)
{
  if(M.size() == 0) {
    std::cout << "[DBG] " << name << ": <empty>\n";
    return;
  }
  const double fro = M.norm();
  std::cout << "[DBG] " << name
            << " rows=" << M.rows()
            << " cols=" << M.cols()
            << " fro_norm=" << fro
            << "\n";
}

void IDProblem::debugPrintBoundsStats(const std::string& name,
                                     const Eigen::VectorXd& l,
                                     const Eigen::VectorXd& u)
{
  if(l.size() == 0 || u.size() == 0) {
    std::cout << "[DBG] " << name << ": <empty bounds>\n";
    return;
  }
  int bad = 0;
  for(int i = 0; i < l.size(); ++i) {
    if(l(i) > u(i)) bad++;
  }
  std::cout << "[DBG] " << name
            << " bounds size=" << l.size()
            << " l[min,max]=[" << l.minCoeff() << ", " << l.maxCoeff() << "]"
            << " u[min,max]=[" << u.minCoeff() << ", " << u.maxCoeff() << "]"
            << " inversions(l>u)=" << bad
            << "\n";
}

void IDProblem::debugDumpSolveStep(const QPProblem* qp,
                                  const Eigen::VectorXd* x,
                                  const Eigen::VectorXd* tau,
                                  bool qp_built,
                                  bool qp_solved,
                                  bool torque_ok) const
{
  if(!debug_enabled_) return;

  std::cout << "\n==================== IDProblem DEBUG DUMP ====================\n";
  std::cout << std::boolalpha
            << "[DBG] qp_built=" << qp_built
            << " qp_solved=" << qp_solved
            << " torque_ok=" << torque_ok
            << "\n";

  // --- robot state
  if(debug_mask_ & DBG_STATE)
  {
    Eigen::VectorXd q, qd;
    q.setZero(model_->getJointNum());
    qd.setZero(model_->getJointNum());

    bool okq = model_->getJointPosition(q);
    bool okd = model_->getJointVelocity(qd);

    std::cout << "[DBG] state: getJointPosition=" << okq
              << " getJointVelocity=" << okd << "\n";
    if(okq) debugPrintVecStats("q", q);
    if(okd) debugPrintVecStats("qd", qd);

    std::cout << "[DBG] regularization_value="<<regularization_value_ <<"\n";
    std::cout << "[DBG] min_forces_weight="<<min_forces_weight_<<"\n";
    std::cout << "[DBG] min_qddot_weight="<<min_qddot_weight_<<"\n";
  }

  // --- qp bounds & constraints
  if((debug_mask_ & DBG_QP_BOUNDS) && qp)
  {
    std::cout << "[DBG] QP: n=" << qp->n() << " m=" << qp->m() << "\n";
    debugPrintMatStats("H", qp->H);
    debugPrintVecStats("g", qp->g);

    debugPrintBoundsStats("var_bounds(l/u)", qp->l, qp->u);

    if(qp->A.rows() > 0) {
      debugPrintMatStats("A", qp->A);
      debugPrintBoundsStats("lin_bounds(lA/uA)", qp->lA, qp->uA);
    } else {
      std::cout << "[DBG] linear constraints: none\n";
    }

    int tight = 0;
    if(x && x->size() == qp->n()) {
      for(int i = 0; i < qp->n(); ++i) {
        const double xi = (*x)(i);
        const double dl = std::abs(xi - qp->l(i));
        const double du = std::abs(qp->u(i) - xi);
        if(dl < 1e-6 || du < 1e-6) tight++;
      }
      std::cout << "[DBG] tight var bounds count ~ " << tight << "/" << qp->n() << "\n";
    }
  }

  // --- solution
  if((debug_mask_ & DBG_SOLUTION) && x)
  {
    debugPrintVecStats("x", *x);

    try {
      const auto& qb = vars_->qddotBlock();
      if(qb.dim > 0 && qb.offset + qb.dim <= x->size()) {
        Eigen::VectorXd x_qdd = x->segment(qb.offset, qb.dim);
        debugPrintVecStats("x[qddot_block]", x_qdd);
      }

      Eigen::VectorXd qdd = x->segment(qb.offset, qb.dim);
      std::cout << "[DBG] qddot_base = " << qdd.head<6>().transpose() << "\n";

      for(const auto& fn : foot_names_) {
        const auto& cb = vars_->contactBlock(fn);
        if(cb.dim > 0 && cb.offset + cb.dim <= x->size()) {
          Eigen::VectorXd x_f = x->segment(cb.offset, cb.dim);
          debugPrintVecStats("x[contact:" + fn + "]", x_f);
        }
      }
    }
    catch(...) {
      std::cout << "[DBG] (note) could not print x blocks (vars_ API mismatch)\n";
    }

    // dynamics residual (assumes last 6 rows are dynamics equality; keep as your debug)
    if(qp && qp->m() >= 6) {
      int m = qp->m();
      Eigen::MatrixXd A_dyn = qp->A.block(m-6, 0, 6, qp->n());
      Eigen::VectorXd l_dyn = qp->lA.segment(m-6, 6);
      Eigen::VectorXd r_qp  = A_dyn * (*x) - l_dyn;

      std::cout << "[DBG] dyn_qp_residual = " << r_qp.transpose()
                << " |r|=" << r_qp.norm() << "\n";
    }
  }

  // --- tau
  if((debug_mask_ & DBG_TAU) && tau)
  {
    debugPrintVecStats("tau", *tau);
    const int N = std::min<int>(tau->size(), 16);
    std::cout << "[DBG] tau head(" << N << ") = " << tau->head(N).transpose() << "\n";
  }

  // --- contacts
  if(debug_mask_ & DBG_CONTACTS)
  {
    if(!contact_wrenches_.empty()) {
      double fz_sum = 0.0;
      for(size_t i = 0; i < contact_wrenches_.size(); ++i) {
        const auto& w = contact_wrenches_[i];
        const double fx = w(0), fy = w(1), fz = w(2);
        fz_sum += fz;
        std::cout << "[DBG] wrench[" << i << "] "
                  << "fx=" << fx << " fy=" << fy << " fz=" << fz
                  << " |f|=" << w.head<3>().norm()
                  << "\n";
      }
      const double weight = model_->getMass() * GRAVITY;
      std::cout << "[DBG] fz_sum=" << fz_sum << " vs weight=" << weight
                << " ratio=" << (weight > 1e-9 ? (fz_sum / weight) : 0.0)
                << "\n";
    } else {
      std::cout << "[DBG] contact_wrenches_: empty (maybe not computed yet)\n";
    }
  }

  // --- tasks snapshot
  if(debug_mask_ & DBG_TASKS)
  {

    auto print_task = [&](const std::string& name, const auto& task){
      std::cout << "[DBG] task: " << name << "\n";
      debugPrintMatStats("  A", task.A());
      debugPrintVecStats("  b", task.b());
      debugPrintMatStats("  W", task.W());
      std::cout << "[DBG]   b=" << task.b().transpose() << "\n";
      std::cout << "[DBG]   W diag=" << task.W().diagonal().transpose() << "\n";

      auto print_diag = [&](const std::string& tag, const Eigen::MatrixXd& M){
        if(M.size() == 0) {
          std::cout << "[DBG]   " << tag << " <empty>\n";
          return;
        }
        if(M.rows() != M.cols()) {
          std::cout << "[DBG]   " << tag << " (" << M.rows() << "x" << M.cols()
                    << ") not square, head=" << M.topLeftCorner(std::min(3,(int)M.rows()),
                                                              std::min(3,(int)M.cols()))
                    << "\n";
          return;
        }
        std::cout << "[DBG]   " << tag << " diag=" << M.diagonal().transpose() << "\n";
      };

      try {
        if(const auto* cart = dynamic_cast<const CartesianImpl*>(&task)) {
          const auto Kp = cart->getKp();
          const auto Kd = cart->getKd();
          print_diag("cartesian Kp", Kp);
          print_diag("cartesian Kd", Kd);
          std::cout << "[DBG]   cartesian gain_type=" << static_cast<int>(cart->getGainType()) << "\n";
        }
        else if(const auto* com = dynamic_cast<const ComImpl*>(&task)) {
          const auto Kp = com->getKp();
          const auto Kd = com->getKd();
          print_diag("com Kp", Kp);
          print_diag("com Kd", Kd);
        }
        else if(const auto* post = dynamic_cast<const PosturalImpl*>(&task)) {
          const auto Kp = post->getKp();
          const auto Kd = post->getKd();
          print_diag("postural Kp", Kp);
          print_diag("postural Kd", Kd);
        }
        else if(const auto* mom = dynamic_cast<const AngularMomentumImpl*>(&task)) {
          const auto Kg = mom->getMomentumGain();
          print_diag("momentum_gain", Kg);
        }
      } catch(...) {
        std::cout << "[DBG]   (note) gains print failed for " << name << "\n";
      }
    };

    try {
      for(const auto& kv : feet_) print_task("foot:" + kv.first, *kv.second);
      for(const auto& kv : arms_) print_task("arm:" + kv.first, *kv.second);
      print_task("waist", *waist_);
      print_task("com", *com_);

      // -------------------- EXTRA COM DEBUG (model-side + task-side) --------------------
      try {
        Eigen::Vector3d com_p, com_v, jdotqdot;
        Eigen::MatrixXd Jcom;

        model_->getCOM(com_p);
        model_->getCOMVelocity(com_v);
        model_->getCOMJacobian(Jcom, jdotqdot); // Jcom: 3 x ndof (o joint num del model)

        std::cout << "[DBG]   [COM_DBG] model COM pos   = " << com_p.transpose() << "\n";
        std::cout << "[DBG]   [COM_DBG] model COM vel   = " << com_v.transpose() << "\n";
        std::cout << "[DBG]   [COM_DBG] model dJcomQdot = " << jdotqdot.transpose() << "\n";

        // Some quick stats on Jcom
        if(Jcom.size() > 0) {
          std::cout << "[DBG]   [COM_DBG] Jcom rows=" << Jcom.rows()
                    << " cols=" << Jcom.cols()
                    << " fro_norm=" << Jcom.norm()
                    << "\n";
        } else {
          std::cout << "[DBG]   [COM_DBG] Jcom: <empty>\n";
        }

        // Task-side: infer "desired accel" y = b + dJcomQdot (since your task builds b = y - dJdq)
        {
          const Eigen::Vector3d b_task = com_->b();  // should be size 3
          std::cout << "[DBG]   [COM_DBG] task b         = " << b_task.transpose() << "\n";
          std::cout << "[DBG]   [COM_DBG] y_est (=b+dJq) = " << (b_task + jdotqdot).transpose() << "\n";
        }

        // If solution x is available, compute actual com acceleration implied by solution:
        if(x && x->size() == vars_->size()) {
          const Eigen::VectorXd qdd = vars_->qddot(*x);

          // Protect against dimension mismatch between Jcom and qdd
          if(Jcom.cols() == qdd.size()) {
            Eigen::Vector3d com_acc = Jcom * qdd + jdotqdot;
            std::cout << "[DBG]   [COM_DBG] com_acc (J*qdd + dJq) = "
                      << com_acc.transpose() << "\n";
          } else {
            std::cout << "[DBG]   [COM_DBG] cannot compute com_acc: Jcom.cols("
                      << Jcom.cols() << ") != qdd.size(" << qdd.size() << ")\n";
          }

          // Also print base part of qddot just for correlation
          if(qdd.size() >= 6) {
            std::cout << "[DBG]   [COM_DBG] qddot_base (from x) = "
                      << qdd.head<6>().transpose() << "\n";
          }
        }

      } catch(...) {
        std::cout << "[DBG]   [COM_DBG] (note) failed to compute model-side COM debug\n";
      }
      // ---------------------------------------------------------------------------------

      print_task("postural", *postural_);
      print_task("angular_momentum", *angular_momentum_);
    }
    catch(...) {
      std::cout << "[DBG] (note) task snapshot failed (API mismatch?)\n";
    }
  }

  if((debug_mask_ & DBG_CONSTRAINTS) && qp)
  {
    std::cout << "[DBG] constraints list (" << constraints_.size() << ")\n";
    for(const auto& c : constraints_)
    {
      if(!c) continue;
      std::cout << "[DBG]   c=" << c->name()
                << " enabled=" << c->enabled()
                << " rows=" << c->rows()
                << " cols=" << c->cols()
                << "\n";
    }
  }

  std::cout << "=============================================================\n\n";
}

} // namespace wolf_wbid
