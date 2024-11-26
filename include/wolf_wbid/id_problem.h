/**
WoLF: WoLF: Whole-body Locomotion Framework for quadruped robots (c) by Gennaro Raiola

WoLF is licensed under a license Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.

You should have received a copy of the license along with this
work. If not, see <http://creativecommons.org/licenses/by-nc-nd/4.0/>.
**/

#ifndef WOLF_WBID_ID_PROBLEM_H
#define WOLF_WBID_ID_PROBLEM_H

// STD
#include <atomic>
#include <mutex>
#include <memory>

// WoLF
#include <wolf_wbid/task_interface.h>
#include <wolf_wbid/quadruped_robot.h>
#include <wolf_controller_utils/common.h>

// OpenSoT
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/AngularMomentum.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/tasks/acceleration/DynamicFeasibility.h>
#include <OpenSoT/tasks/MinimizeVariable.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/constraints/acceleration/TorqueLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/utils/InverseDynamics.h>
#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/constraints/force/WrenchLimits.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/constraints/acceleration/JointLimits.h>

namespace wolf_wbid {

/**
 * @brief The IDProblem class wraps the tasks, constraints and the solver used to solve the inverse dynamic problem (ID)
 */
class IDProblem
{

public:

    enum mode_t {WPG=0,EXT,MPC};

    const std::string CLASS_NAME = "IDProblem";

    typedef std::shared_ptr<IDProblem> Ptr;

    typedef std::unique_ptr<IDProblem> UniquePtr;

    /**
     * @brief IDProblem constructor
     * @param model pointer to external model
     */
    IDProblem(QuadrupedRobot::Ptr model);

    /**
     * @brief IDProblem destructor
     */
    ~IDProblem();

    /**
     * @brief initialize the IDProblem (after creating the tasks)
     * @param robot_name
     * @param dt
     */
    void init(const std::string& robot_name, const double& dt);

    /**
     * @brief solve call this after update()
     * @param x solution form solver (tau)
     * @return true if solved
     */
    bool solve(Eigen::VectorXd& x);

    /**
     * @brief get the contact wrenches, to call after solve()
     */
    const std::vector<Eigen::Vector6d>& getContactWrenches() const;

    /**
     * @brief get the joint accelerations, to call after solve()
     */
    const Eigen::VectorXd& getJointAccelerations() const;

    /**
     * @brief swing with a specific foot i.e. release the contact
     */
    void swingWithFoot(const std::string& foot_name, const std::string &ref_frame);

    /**
     * @brief stance with a specific foot i.e. activate the contact
     */
    void stanceWithFoot(const std::string& foot_name, const std::string& ref_frame);

    /**
     * @brief publish the ros topics related to the tasks
     */
    void publish();

    /**
     * @brief set the postural reference and gains
     * @param Kp
     * @param Kd
     * @param q
     */
    void setPosture(const Eigen::MatrixXd &Kp, const Eigen::MatrixXd &Kd, const Eigen::VectorXd &q);

    /**
     * @brief set the mu parameter for the friction cones
     * @param mu value
     */
    void setFrictionConesMu(const double& mu);

    /**
     * @brief set the rotation matrix for the friction cones
     * @param R 3d matrix
     */
    void setFrictionConesR(const Eigen::Matrix3d& R);

    /**
     * @brief set the pose and velocity reference for the feet,
     * note that the foot tasks are defined wrt the base frame
     * @param foot_name foot name
     * @param pose_ref pose reference
     * @param vel_ref velocity reference (note: we use only the linear part of it)
     * @param reference_frame if world tranform the references into base frame
     */
    void setFootReference(const std::string& foot_name, const Eigen::Affine3d& pose_ref, const Eigen::Vector6d& vel_ref, const std::string& reference_frame);

    /**
     * @brief set the lower bound for the wrench limits along the selected axis (w.r.t world)
     * @param lower bound values
     */
    void setLowerForceBound(const double& x_force,const double& y_force,const double& z_force);

    /**
     * @brief setLowerForceBoundX
     * @param force
     */
    void setLowerForceBoundX(const double& force);

    /**
     * @brief setLowerForceBoundY
     * @param force
     */
    void setLowerForceBoundY(const double& force);

    /**
     * @brief setLowerForceBoundZ
     * @param force
     */
    void setLowerForceBoundZ(const double& force);

    /**
     * @brief set the angular reference and height for the waist (aka base)
     * @param Rot rotation matrix
     * @param z height
     */
    void setWaistReference(const Eigen::Matrix3d &Rot, const double &z, const double &z_vel);

    /**
     * @brief set the position and velocity reference for the CoM
     * @param position
     * @param velocity
     */
    void setComReference(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity);

    /**
     * @brief set the control mode
     */
    void setControlMode(mode_t mode);

    /**
     * @brief get the mu parameter for the friction cones
     */
    double getFrictionConesMu() const;

    /**
     * @brief reset the tasks
     */
    void reset();

    /**
     * @brief if true control the height of the robot via the CoM z position, otherwise
     * use the waist z (default true)
     */
    void activateComZ(bool active);

    /**
     * @brief if true activate the angular momentum task (default true)
     */
    void activateAngularMomentum(bool active);

    /**
     * @brief if true activate the postural task (default false)
     */
    void activatePostural(bool active);

    /**
     * @brief if true activate the joint position limits constraint (default false)
     */
    void activateJointPositionLimits(bool active);

    /**
     * @brief set the regularization value for the solver,
     * note that the forces regularization value will be calculated as reg*reg
     * (default 1e-3)
     */
    void setRegularization(double regularization);

    /**
     * @brief set the weight for the force minimization tasks, i.e. higher values will keep the contact forces as lower as possible
     */
    void setForcesMinimizationWeight(double weight);

    /**
     * @brief set the weight for the joint acceleration minimization tasks, i.e. higher values will keep the qddot as lower as possible
     */
    void setJointAccelerationMinimizationWeight(double weight);


    /** Get **/
    const std::map<std::string,Cartesian::Ptr>& getFootTasks() const;
    const std::map<std::string,Cartesian::Ptr>& getArmTasks() const;
    const std::map<std::string,Wrench::Ptr>& getWrenchTasks() const;
    const Cartesian::Ptr & getWaistTask() const;
    const Com::Ptr & getComTask() const;

private:

    bool _solve(const std::unique_ptr<OpenSoT::solvers::iHQP>& solver, Eigen::VectorXd& tau);

    void activateExternalReferences(bool activate);

    /**
     * @brief Tasks
     */
    std::map<std::string,Cartesian::Ptr> feet_;
    std::map<std::string,Cartesian::Ptr> arms_;
    std::map<std::string,Wrench::Ptr> wrenches_;
    Cartesian::Ptr waist_;
    Com::Ptr com_;
    AngularMomentum::Ptr angular_momentum_;
    OpenSoT::tasks::GenericTask::Ptr regularization_;
    std::vector<OpenSoT::tasks::MinimizeVariable::Ptr> min_forces_;
    OpenSoT::tasks::MinimizeVariable::Ptr min_qddot_;
    OpenSoT::tasks::acceleration::DynamicFeasibility::Ptr dynamics_task_;
    OpenSoT::constraints::TaskToConstraint::Ptr dynamics_con_;

    /**
     * @brief postural_ a postural task
     */
    Postural::Ptr postural_;

    /**
     * @brief torque_lims_ some bounds
     */
    OpenSoT::constraints::acceleration::TorqueLimits::Ptr torque_lims_;

    /**
     * @brief wrenches_lims_ bounds
     */
    OpenSoT::constraints::force::WrenchesLimits::Ptr wrenches_lims_;

    /**
     * @brief q_lims_ bounds
     */
    OpenSoT::constraints::acceleration::JointLimits::Ptr q_lims_;

    /**
     * @brief _model
     */
    QuadrupedRobot::Ptr model_;

    /**
     * @brief update call after the model.update() to update the autostack
     */
    void update();

    /**
     * @brief friction_cones_ constraints
     */
    OpenSoT::constraints::force::FrictionCones::Ptr friction_cones_;

    /**
     * @brief map of stacks for the Walkin Pattern Generator (WPG)
     */
    OpenSoT::AutoStack::Ptr wpg_stack_;

    /**
     * @brief map of stacks for the Model Predictive Controller (MPC)
     */
    OpenSoT::AutoStack::Ptr mpc_stack_;

    /**
     * @brief solver_ iHQP solver
     */
    std::unique_ptr<OpenSoT::solvers::iHQP> wpg_solver_;

    /**
     * @brief solver_ iHQP solver
     */
    std::unique_ptr<OpenSoT::solvers::iHQP> mpc_solver_;

    /**
     * @brief id_ inverse dynamics computation & variable helper
     */
    OpenSoT::utils::InverseDynamics::Ptr id_;

    /**
     * @brief x_ full solver solution
     */
    Eigen::VectorXd x_;

    /**
     * @brief qddot_ joint accelerations solution
     */
    Eigen::VectorXd qddot_;

    /**
     * @brief contact_wrenches_ contacts solution
     */
    std::vector<Eigen::Vector6d> contact_wrenches_;

    /**
     * @brief wrench limitis
     */
    Eigen::Vector6d wrench_upper_lims_;
    Eigen::Vector6d wrench_lower_lims_;

    /**
     * @brief joint acceleration limits
     */
    Eigen::VectorXd ones_;
    std::atomic<double> joint_acceleration_lim_;

    /**
     * @brief Force lower limits
     */
    double x_force_lower_lim_;
    double y_force_lower_lim_;
    double z_force_lower_lim_;

    /**
     * @brief Friction cones
     */
    OpenSoT::constraints::force::FrictionCone::friction_cone fc_;

    /**
     * @brief Flags and variables
     */
    std::atomic<unsigned int> control_mode_;
    bool initialized_;
    bool activate_com_z_;
    bool activate_angular_momentum_;
    bool activate_postural_;
    bool activate_joint_position_limits_;
    bool change_control_mode_;
    double regularization_value_;
    double min_forces_weight_;
    double min_qddot_weight_;

    /**
     * @brief Various names
     */
    std::vector<std::string> foot_names_;
    std::vector<std::string> ee_names_;
    std::vector<std::string> contact_names_;

    /**
     * @brief Temporary variables
     */
    Eigen::Affine3d tmp_affine3d_;
    Eigen::Vector6d tmp_vector6d_;
    Eigen::Vector3d tmp_vector3d_;
    Eigen::Vector3d tmp_vector3d_1_;

};

} // namespace

#endif

