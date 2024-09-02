#include <XBotInterface/RobotInterface.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

namespace py = pybind11;
using namespace XBot;

Eigen::VectorXd q;
Eigen::VectorXd q_dot;
JointNameMap qmap;
Eigen::MatrixXd J, J3;
Eigen::MatrixXd M;
Eigen::Vector3d v3;


auto sync_from = [](ModelInterface& self, const XBotInterface& other)
{
    self.syncFrom(other, XBot::Sync::All);
};

auto getjointpos = [](const XBotInterface& self)
{
    self.getJointPosition(q);
    return q;
};

auto getjointvel = [](const XBotInterface& self)
{
    self.getJointVelocity(q_dot);
    return q_dot;
};

auto getjointacc = [](const XBotInterface& self)
{
    self.getJointAcceleration(q_dot);
    return q_dot;
};

auto getposref = [](const XBotInterface& self)
{
    self.getPositionReference(q);
    return q;
};

auto getvelref = [](const XBotInterface& self)
{
    self.getVelocityReference(q);
    return q;
};

auto gettorref = [](const XBotInterface& self)
{
    self.getEffortReference(q);
    return q;
};

auto getmotpos = [](const XBotInterface& self)
{
    self.getMotorPosition(q);
    return q;
};

auto getmotvel = [](const XBotInterface& self)
{
    self.getMotorVelocity(q);
    return q;
};

auto getjointtor = [](const XBotInterface& self)
{
    self.getJointEffort(q);
    return q;
};

auto getjointk = [](const XBotInterface& self)
{
    self.getStiffness(q);
    return q;
};

auto getjointd = [](const XBotInterface& self)
{
    self.getDamping(q);
    return q;
};

auto getjointpos_map = [](const XBotInterface& self)
{
    self.getJointPosition(qmap);
    return qmap;
};

auto getjointtor_map = [](const XBotInterface& self)
{
    self.getJointEffort(qmap);
    return qmap;
};

auto getjointk_map = [](const XBotInterface& self)
{
    self.getStiffness(qmap);
    return qmap;
};

auto getjointd_map = [](const XBotInterface& self)
{
    self.getDamping(qmap);
    return qmap;
};

auto getrobot = [](const ConfigOptions& opt)
{
    auto robot_shptr = RobotInterface::getRobot(opt);
    RobotInterface::clearRobotMap();
    return robot_shptr;
};

auto getmodel = [](const ConfigOptions& opt)
{
    auto robot_shptr = ModelInterface::getModel(opt);
    return robot_shptr;
};

auto getgravity = [](const ModelInterface& self)
{
    self.getGravity(v3);
    return v3;
};

auto getjacobian = [](const ModelInterface& self, std::string link)
{
    self.getJacobian(link, J);
    return J;
};

auto getcomjacobian = [](const ModelInterface& self)
{
    self.getCOMJacobian(J3);
    return J3;
};

auto getreljacobian = [](const ModelInterface& self, std::string source, std::string target)
{
    self.getRelativeJacobian(source, target, J);
    return J;
};

auto modelupdate = [](ModelInterface& self)
{
    self.update();
};

auto gcomp = [](ModelInterface& self)
{
    self.computeGravityCompensation(q);
    return q;
};

auto nl = [](ModelInterface& self)
{
    self.computeNonlinearTerm(q);
    return q;
};

auto id = [](ModelInterface& self)
{
    self.computeInverseDynamics(q);
    return q;
};

auto getm = [](ModelInterface& self)
{
    self.getInertiaMatrix(M);
    return M;
};

auto eigen2map = [](XBotInterface& self, const Eigen::VectorXd& qe)
{
    self.eigenToMap(qe, qmap);
    return qmap;
};

auto map2eigen = [](XBotInterface& self, const JointNameMap& qm)
{
    self.mapToEigen(qm, q);
    return q;
};

auto getpose = [](ModelInterface& self, std::string link)
{
    Eigen::Affine3d T;
    self.getPose(link, T);
    return T;
};

auto getfloatingbasepose = [](ModelInterface& self)
{
    Eigen::Affine3d T;
    self.getFloatingBasePose(T);
    return T;
};

auto getpose_1 = [](ModelInterface& self, std::string src, std::string tgt)
{
    Eigen::Affine3d T;
    self.getPose(src, tgt, T);
    return T;
};

auto getcom = [](ModelInterface& self)
{
    Eigen::Vector3d com;
    self.getCOM(com);
    return com;
};

auto getcomvel = [](ModelInterface& self)
{
    Eigen::Vector3d com_vel;
    self.getCOMVelocity(com_vel);
    return com_vel;
};

auto getwrench = [](ForceTorqueSensor& self)
{
    Eigen::Vector6d w;
    self.getWrench(w);
    return w;
};

auto getorientation = [](ImuSensor& self)
{
    Eigen::Matrix3d R;
    self.getOrientation(R);
    return R;
};

auto getangvel = [](ImuSensor& self)
{
    Eigen::Vector3d w;
    self.getAngularVelocity(w);
    return w;
};

auto getlinacc = [](ImuSensor& self)
{
    Eigen::Vector3d w;
    self.getLinearAcceleration(w);
    return w;
};

auto getjointkChain = [](KinematicChain& self)
{
    self.getStiffness(q);
    return q;
};

auto getjointkChainMap = [](KinematicChain& self)
{
    self.getStiffness(qmap);
    return qmap;
};

auto getjointdChain = [](KinematicChain& self)
{
    self.getDamping(q);
    return q;
};

auto getjointdChainMap = [](KinematicChain& self)
{
    self.getDamping(qmap);
    return qmap;
};

auto getcmm = [](const ModelInterface& self)
{
    Eigen::MatrixXd cmm;
    Eigen::Vector6d dcmm;
    self.getCentroidalMomentumMatrix(cmm, dcmm);

    return std::make_tuple(cmm, dcmm);
};

auto getjlim = [](const XBotInterface& self)
{
    Eigen::VectorXd qmin, qmax;
    self.getJointLimits(qmin, qmax);
    return std::make_pair(qmin, qmax);
};

auto getvlim = [](const XBotInterface& self)
{
    Eigen::VectorXd dq;
    self.getVelocityLimits(dq);
    return dq;
};

auto gettorlim = [](const XBotInterface& self)
{
    Eigen::VectorXd tau;
    self.getEffortLimits(tau);
    return tau;
};

auto getrobstate= [](const XBotInterface& self,
        std::string s)
{
    Eigen::VectorXd q;
    if(!self.getRobotState(s, q))
    {
        throw py::key_error();
    }
    return q;
};

auto get_vel_twist = [](const ModelInterface& self, const std::string& l)
{
    Eigen::Vector6d v;
    if(!self.getVelocityTwist(l, v))
    {
        throw py::key_error();
    }
    return v;
};

auto get_rel_vel_twist = [](const ModelInterface& self, const std::string& l, const std::string& base)
{
    Eigen::Vector6d v;
    if(!self.getRelativeVelocityTwist(l, base, v))
    {
        throw py::key_error();
    }
    return v;
};

PYBIND11_MODULE(xbot_interface, m) {
    
    py::class_<XBotInterface, std::shared_ptr<XBotInterface>>(m, "XBotInterface")
            .def("getUrdfString", &XBotInterface::getUrdfString)
            .def("getSrdfString", &XBotInterface::getSrdfString)
            .def("getJointNum", &XBotInterface::getJointNum)
            .def("getJointLimits", getjlim)
            .def("getVelocityLimits", getvlim)
            .def("getEffortLimits", gettorlim)
            .def("getRobotState", getrobstate)
            .def("getJointPosition", getjointpos)
            .def("getJointEffort", getjointtor)
            .def("getJointPositionMap", getjointpos_map)
            .def("getJointEffortMap", getjointtor_map)
            .def("getJointVelocity", getjointvel)
            .def("getJointAcceleration", getjointacc)
            .def("getJointByName", &XBotInterface::getJointByName, py::return_value_policy::reference)
            .def("getJointByDofIndex", &XBotInterface::getJointByDofIndex, py::return_value_policy::reference)
            .def("getJointByID", &XBotInterface::getJointByID, py::return_value_policy::reference)
            .def("getEnabledJointNames", &XBotInterface::getEnabledJointNames)
            .def("getEnabledJointId", &XBotInterface::getEnabledJointId)
            .def("getDofIndex", (int (XBotInterface::*)(const std::string&) const) &XBotInterface::getDofIndex)
            .def("getDofIndex", (int (XBotInterface::*)(int) const) &XBotInterface::getDofIndex)
            .def("eigenToMap", eigen2map)
            .def("mapToEigen", map2eigen)
            .def("getForceTorque", (std::map<std::string, ForceTorqueSensor::ConstPtr> (XBotInterface::*)() const) &XBotInterface::getForceTorque, py::return_value_policy::reference)
            .def("getImu", (std::map<std::string, ImuSensor::ConstPtr> (XBotInterface::*)()) &XBotInterface::getImu, py::return_value_policy::reference)
            ;

    py::class_<RobotInterface, XBotInterface, std::shared_ptr<RobotInterface>>(m, "RobotInterface")
            .def(py::init(getrobot), py::return_value_policy::reference)
            .def("sense", &RobotInterface::sense, py::arg("update_model") = true)
            .def("move", &RobotInterface::move)
            .def("getMotorPosition", getmotpos)
            .def("getMotorVelocity", getmotvel)
            .def("getStiffness", getjointk)
            .def("getDamping", getjointd)
            .def("getStiffnessMap", getjointk_map)
            .def("getDampingMap", getjointd_map)
            .def("getPositionReference", getposref)
            .def("getVelocityReference", getvelref)
            .def("getEffortReference", gettorref)
            .def("setPositionReference", (bool (XBotInterface::*)(const Eigen::VectorXd&)) &XBotInterface::setPositionReference)
            .def("setPositionReference", (bool (XBotInterface::*)(const JointNameMap&)) &XBotInterface::setPositionReference)
            .def("setVelocityReference", (bool (XBotInterface::*)(const Eigen::VectorXd&)) &XBotInterface::setVelocityReference)
            .def("setVelocityReference", (bool (XBotInterface::*)(const JointNameMap&)) &XBotInterface::setVelocityReference)
            .def("setEffortReference", (bool (XBotInterface::*)(const Eigen::VectorXd&)) &XBotInterface::setEffortReference)
            .def("setEffortReference", (bool (XBotInterface::*)(const JointNameMap&)) &XBotInterface::setEffortReference)
            .def("setDamping", (bool (XBotInterface::*)(const Eigen::VectorXd&)) &XBotInterface::setDamping)
            .def("setDamping", (bool (XBotInterface::*)(const JointNameMap&)) &XBotInterface::setDamping)
            .def("setStiffness", (bool (XBotInterface::*)(const Eigen::VectorXd&)) &XBotInterface::setStiffness)
            .def("setStiffness", (bool (XBotInterface::*)(const JointNameMap&)) &XBotInterface::setStiffness)
            .def("setControlMode", (bool (RobotInterface::*)(const ControlMode&)) &RobotInterface::setControlMode)
            .def("setControlMode", (bool (RobotInterface::*)(const std::string&, const ControlMode&)) &RobotInterface::setControlMode)
            .def("setControlMode", (bool (RobotInterface::*)(const std::map<std::string, ControlMode>&)) &RobotInterface::setControlMode)
            .def("model", &RobotInterface::model, py::return_value_policy::reference)
            .def("leg", (XBot::RobotChain& (RobotInterface::*) (int)) &RobotInterface::leg, py::return_value_policy::reference)
            .def("leg", (const XBot::RobotChain& (RobotInterface::*) (int) const) &RobotInterface::leg, py::return_value_policy::reference)
            .def("arm", (XBot::RobotChain& (RobotInterface::*) (int)) &RobotInterface::arm, py::return_value_policy::reference)
            .def("arm", (const XBot::RobotChain& (RobotInterface::*) (int) const) &RobotInterface::arm, py::return_value_policy::reference)
            ;

    py::class_<ModelInterface, XBotInterface, std::shared_ptr<ModelInterface>>(m, "ModelInterface")
            .def(py::init(getmodel), py::return_value_policy::reference)
            .def("syncFrom", sync_from)
            .def("getJacobian", getjacobian)
            .def("getCOMJacobian", getcomjacobian)
            .def("getRelativeJacobian", getreljacobian)
            .def("getGravity", getgravity)
            .def("setJointPosition", (bool (XBotInterface::*)(const Eigen::VectorXd&)) &XBotInterface::setJointPosition)
            .def("setJointPosition", (bool (XBotInterface::*)(const JointNameMap&)) &XBotInterface::setJointPosition)
            .def("setJointVelocity", (bool (XBotInterface::*)(const Eigen::VectorXd&)) &XBotInterface::setJointVelocity)
            .def("setJointVelocity", (bool (XBotInterface::*)(const JointNameMap&)) &XBotInterface::setJointVelocity)
            .def("setJointEffort", (bool (XBotInterface::*)(const Eigen::VectorXd&)) &XBotInterface::setJointEffort)
            .def("setJointEffort", (bool (XBotInterface::*)(const JointNameMap&)) &XBotInterface::setJointEffort)
            .def("setJointAcceleration", (bool (XBotInterface::*)(const Eigen::VectorXd&)) &XBotInterface::setJointAcceleration)
            .def("setJointAcceleration", (bool (XBotInterface::*)(const JointNameMap&)) &XBotInterface::setJointAcceleration)
            .def("update", modelupdate)
            .def("computeGravityCompensation", gcomp)
            .def("computeNonlinearTerm", nl)
            .def("computeInverseDynamics", id)
            .def("getVelocityTwist", get_vel_twist)
            .def("getRelativeVelocityTwist", get_rel_vel_twist)
            .def("getPose", getpose)
            .def("getPose", getpose_1)
            .def("getMass", &ModelInterface::getMass)
            .def("getCOM", getcom)
            .def("getCOMVelocity", getcomvel)
            .def("getInertiaMatrix", getm)
            .def("getCentroidalMomentumMatrix", getcmm)
            .def("getFloatingBasePose", getfloatingbasepose)
            .def("setFloatingBasePose", (bool (ModelInterface::*)(const Eigen::Affine3d&))&ModelInterface::setFloatingBasePose)
            .def("setFloatingBaseState", (bool (ModelInterface::*)(const Eigen::Affine3d&, const Eigen::Vector6d&))&ModelInterface::setFloatingBaseState)
            .def("setFloatingBaseState", (bool (ModelInterface::*)(ImuSensor::ConstPtr, const Eigen::Matrix3d&)) &ModelInterface::setFloatingBaseState,
                 py::arg("imu"), py::arg("R0") = Eigen::Matrix3d::Identity())
            .def("setGravity", (void (ModelInterface::*)(const Eigen::Vector3d&)) &ModelInterface::setGravity)
            ;
    
    py::class_<Joint>(m, "Joint")
            .def("getJointName", &Joint::getJointName)
            .def("getJointID", &Joint::getJointId)
            .def("printState", &Joint::print)
            ;

    py::class_<GenericSensor, std::shared_ptr<GenericSensor>>(m, "GenericSensor")
            .def("getSensorName", &GenericSensor::getSensorName)
            ;

    py::class_<ForceTorqueSensor, GenericSensor, std::shared_ptr<ForceTorqueSensor>>(m, "ForceTorqueSensor")
            .def("getWrench", getwrench)
            ;

    py::class_<ImuSensor, GenericSensor, std::shared_ptr<ImuSensor>>(m, "ImuSensor")
            .def("getOrientation", getorientation)
            .def("getAngularVelocity", getangvel)
            .def("getLinearAcceleration", getlinacc)
            ;

    py::class_<ControlMode>(m, "ControlMode")
            .def_static("Position", &ControlMode::Position)
            .def_static("Velocity", &ControlMode::Velocity)
            .def_static("Effort", &ControlMode::Effort)
            .def_static("PosImpedance", &ControlMode::PosImpedance)
            .def_static("Stiffness", &ControlMode::Stiffness)
            .def_static("Damping", &ControlMode::Damping)
            .def_static("Idle", &ControlMode::Idle)
            .def(py::self + py::self)
            ;

    py::class_<KinematicChain>(m, "KinematicChain")
            .def(py::init(), py::return_value_policy::reference)
            .def(py::init<const std::string&, const XBot::XBotCoreModel&>(), py::return_value_policy::reference)
            .def(py::init<const std::string&>(), py::return_value_policy::reference)
            .def(py::init<const KinematicChain>(), py::return_value_policy::reference)
            .def("getJointName", &KinematicChain::getJointName)
            .def("getJointNames", &KinematicChain::getJointNames)
            .def("getBaseLinkName", &KinematicChain::getBaseLinkName)
            .def("getJointNum", &KinematicChain::getJointNum)
            .def("getChainName", &KinematicChain::getChainName)
            .def("getStiffness", getjointkChain)
            .def("getStiffness", getjointkChainMap)
            .def("getDamping", getjointdChain)
            .def("getDamping", getjointdChainMap)
            .def("setStiffness", (bool (KinematicChain::*)(const Eigen::VectorXd&)) &KinematicChain::setStiffness)
            .def("setStiffness", (bool (KinematicChain::*)(const JointIdMap&)) &KinematicChain::setStiffness)
            .def("setStiffness", (bool (KinematicChain::*)(const JointNameMap&)) &KinematicChain::setStiffness)
            .def("setStiffness", (bool (KinematicChain::*)(int, double)) &KinematicChain::setStiffness)
            .def("setDamping", (bool (KinematicChain::*)(const Eigen::VectorXd&)) &KinematicChain::setDamping)
            .def("setDamping", (bool (KinematicChain::*)(const JointIdMap&)) &KinematicChain::setDamping)
            .def("setDamping", (bool (KinematicChain::*)(const JointNameMap&)) &KinematicChain::setDamping)
            .def("setDamping", (bool (KinematicChain::*)(int, double)) &KinematicChain::setDamping)
            ;

    py::class_<RobotChain, KinematicChain>(m, "RobotChain")
            .def(py::init(), py::return_value_policy::reference)
            ;
}



