/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <ModelInterfaceRBDL/ModelInterfaceRBDL.h>
#include <Eigen/QR>
#include <cstdio>
#include <sys/stat.h> 
#include <fcntl.h>
#include <XBotInterface/SoLib.h>

#include <XBotInterface/RtLog.hpp>

REGISTER_SO_LIB_(XBot::ModelInterfaceRBDL, XBot::ModelInterface);

using XBot::Logger;


bool XBot::ModelInterfaceRBDL::init_model(const XBot::ConfigOptions& cfg)
{
//     Logger::info() << "Initializing RBDL model using config file : " << path_to_cfg << Logger::endl();
    Logger::info() << "Floating base model: " << (isFloatingBase() ? "TRUE" : "FALSE") << Logger::endl();
    // Init rbdl model with urdf
    if(!RigidBodyDynamics::Addons::URDFReadFromString(getUrdfString().c_str(), &_rbdl_model, isFloatingBase(), false)){
        Logger::error() << "in " << __func__ << ": RBDL model could not be initilized from given URDF string!" << Logger::endl();
        return false;
    }
   
    // Init configuration vectors
    _ndof = _rbdl_model.dof_count;
    _q.setZero(_ndof);
    _qdot.setZero(_ndof);
    _qddot.setZero(_ndof);
    _tau.setZero(_ndof);
    _zeros.setZero(_ndof);
    _jtmp.setZero(_ndof);
    _tmp_jacobian3.setZero(3, _ndof);
    _tmp_jacobian6.setZero(6, _ndof);
    _row_inversion << Eigen::Matrix3d::Zero(),     Eigen::Matrix3d::Identity(),
                      Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero()    ;


    // Fill model-ordered joint id vector
    _model_ordered_joint_names.resize(_ndof);
    int joint_idx = 0;

    _floating_base_link_id = 0;
    _floating_base_link = "";

    if( isFloatingBase() ){
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_1";
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_2";
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_3";
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_4";
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_5";
        _model_ordered_joint_names[joint_idx++] = "VIRTUALJOINT_6";

        _floating_base_link_id = 2;
        _floating_base_link = _rbdl_model.GetBodyName(_floating_base_link_id);
        
        Logger::info() << "Floating base link name: " << _floating_base_link << Logger::endl();
    }

    int link_id_offset = isFloatingBase() ? 2 : 0;
    int dof_num_offset = isFloatingBase() ? -6 : 0;
    for(int i = 0 ; i < (_ndof + dof_num_offset); i++){

        std::string body_name(_rbdl_model.GetBodyName(i+link_id_offset+1));
        urdf::LinkConstSharedPtr link_ptr = getUrdf().getLink(body_name);
        if(!link_ptr){
            Logger::error() << "in " << __func__ << "! Link " << body_name << " NOT defined in the URDF! Check that is_floating_base flag in the config file matches with the first urdf joint type ('floating' if true, 'fixed' if false)" << Logger::endl();
            return false;
        }
        std::string joint_name = link_ptr->parent_joint->name;
        int joint_model_id = _rbdl_model.mJoints[i+link_id_offset+1].q_index;

        Logger::info() << "Joint name: " << joint_name << " RBDL ID: " << joint_model_id << Logger::endl();

        _model_ordered_joint_names[joint_model_id] = joint_name;


    }

    // Fill robot mass
    RigidBodyDynamics::Utils::CalcCenterOfMass(_rbdl_model, _q, _qdot, _mass, _tmp_vector3d, nullptr, nullptr, false);
    
//     std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(_rbdl_model) << std::endl;

    _fb_origin_offset = RigidBodyDynamics::CalcBodyToBaseCoordinates(_rbdl_model, _q*0, _floating_base_link_id, Eigen::Vector3d::Zero(), true);
    if(isFloatingBase()){
        Logger::info() << "Floating base origin offset: " << _fb_origin_offset.transpose() << Logger::endl();
    }

    Logger::success() << "ModelInterfaceRBDL initialized successfully!" << Logger::endl();

    return true;
}

bool XBot::ModelInterfaceRBDL::setFloatingBaseTwist(const KDL::Twist& floating_base_twist)
{
    if(!isFloatingBase()){
        Logger::error() << "in " << __func__ << "! Model is NOT floating base!" << Logger::endl();
        return false;
    }

    _tmp_jacobian6.setZero(6, _rbdl_model.qdot_size);

    Eigen::Vector6d fb_twist_eigen;
    tf::twistKDLToEigen(floating_base_twist, fb_twist_eigen);


    RigidBodyDynamics::CalcPointJacobian6D(_rbdl_model, _q, _floating_base_link_id, Eigen::Vector3d::Zero(), _tmp_jacobian6, false);


    Eigen::Matrix3d Tphi = _tmp_jacobian6.block<3, 3>(0, 3);
    _qdot.segment<3>(3) = Tphi.colPivHouseholderQr().solve(fb_twist_eigen.tail<3>());
    _qdot.head<3>() = fb_twist_eigen.head<3>();

    chain("virtual_chain").setJointVelocity(_qdot.head(6));

    return true;
}


void XBot::ModelInterfaceRBDL::getCOM(KDL::Vector& com_position) const
{
    double mass;
    RigidBodyDynamics::Utils::CalcCenterOfMass(_rbdl_model, _q, _qdot, mass, _tmp_vector3d, nullptr, nullptr, false);
    tf::vectorEigenToKDL(_tmp_vector3d, com_position);
}

void XBot::ModelInterfaceRBDL::getCOMJacobian(KDL::Jacobian& J) const
{
    _tmp_jacobian6.setZero(6, _rbdl_model.dof_count);

    double mass = 0;
    int body_id = 0;

    for( body_id = 1; body_id < _rbdl_model.mBodies.size(); body_id++ ){

        const RigidBodyDynamics::Body& body = _rbdl_model.mBodies[body_id];

        _tmp_jacobian3.setZero(3, _rbdl_model.dof_count);

        RigidBodyDynamics::CalcPointJacobian(_rbdl_model, _q, body_id, body.mCenterOfMass, _tmp_jacobian3, false);

        _tmp_jacobian6.block(0,0,3,_rbdl_model.dof_count) += body.mMass * _tmp_jacobian3;

        mass += body.mMass;

    }

    _tmp_jacobian6 /= mass;
    J.data = _tmp_jacobian6;

}

bool XBot::ModelInterfaceRBDL::getPose(const std::string& source_frame, KDL::Frame& pose) const
{

    int body_id = linkId(source_frame);
    if( body_id == -1 ){
        Logger::error() << "in " << __func__ << ": link " << source_frame << " not defined in RBDL model!" << Logger::endl();
        return false;
    }

    _tmp_vector3d.setZero();

    _tmp_matrix3d = RigidBodyDynamics::CalcBodyWorldOrientation(_rbdl_model,
                                                                _q,
                                                                body_id,
                                                                false);

    _tmp_vector3d = RigidBodyDynamics::CalcBodyToBaseCoordinates(_rbdl_model,
                                                                 _q,
                                                                 body_id,
                                                                 _tmp_vector3d,
                                                                 false);

    _tmp_matrix3d.transposeInPlace();

    rotationEigenToKDL(_tmp_matrix3d, pose.M);
    tf::vectorEigenToKDL(_tmp_vector3d, pose.p);

    return true;

}

int XBot::ModelInterfaceRBDL::linkId(const std::string& link_name) const
{
    int body_id;

    if(link_name == "world"){
        body_id = _rbdl_model.GetBodyId("ROOT");
    }
    else{
        body_id = _rbdl_model.GetBodyId(link_name.c_str());
    }

    if( std::numeric_limits<unsigned int>::max() ==  body_id ){
        return -1;
    }
    else{
        return body_id;
    }
}

bool XBot::ModelInterfaceRBDL::update_internal(bool update_position, bool update_velocity, bool update_desired_acceleration)
{
    bool success = true;
    Eigen::VectorXd *q_ptr = nullptr, *qdot_ptr = nullptr, *qddot_ptr = nullptr;
    if(update_position){
        success = getJointPosition(_q) && success;
        q_ptr = &_q;
    }
    if(update_velocity){
        success = getJointVelocity(_qdot) && success;
        qdot_ptr = &_qdot;
    }
    if(update_desired_acceleration){
        success = getJointAcceleration(_qddot) && success;
        qddot_ptr = &_qddot;
    }

    RigidBodyDynamics::UpdateKinematics(_rbdl_model, _q, _qdot, _qddot);


    return success;
}

bool XBot::ModelInterfaceRBDL::getJacobian(const std::string& link_name,
                                           const KDL::Vector& reference_point,
                                           KDL::Jacobian& J) const
{
    int body_id = linkId(link_name);
    if( body_id == -1 ){
        Logger::error() << "in " << __func__ << ": link " << link_name << " not defined in RBDL model!" << Logger::endl();
        return false;
    }

    _tmp_jacobian6.setZero(6, _rbdl_model.dof_count);
    tf::vectorKDLToEigen(reference_point, _tmp_vector3d);

    RigidBodyDynamics::CalcPointJacobian6D(_rbdl_model, _q, body_id, _tmp_vector3d, _tmp_jacobian6, false);

//     J.data.noalias() = _row_inversion * _tmp_jacobian6;

    J.data.resize(6, _rbdl_model.dof_count);

    J.data.topRows(3) = _tmp_jacobian6.bottomRows(3);
    J.data.bottomRows(3) = _tmp_jacobian6.topRows(3);

    return true;
}

void XBot::ModelInterfaceRBDL::getModelOrderedJoints(std::vector< std::string >& joint_name) const
{
    joint_name = _model_ordered_joint_names;
}

int XBot::ModelInterfaceRBDL::jointModelId(const std::string& joint_name) const
{
    return _rbdl_model.mJoints[linkId(this->getUrdf().getJoint(joint_name)->child_link_name)].q_index;
}


bool XBot::ModelInterfaceRBDL::setFloatingBasePose(const KDL::Frame& floating_base_pose)
{
    if(!isFloatingBase()){
        Logger::error() << "in " << __func__ << "! Model is NOT floating base!" << Logger::endl();
        return false;
    }

    rotationKDLToEigen(floating_base_pose.M, _tmp_matrix3d);
    tf::vectorKDLToEigen(floating_base_pose.p, _tmp_vector3d);

    Eigen::AngleAxisd::RotationMatrixType aa_rot(_tmp_matrix3d);
    _q.segment(3,3) = aa_rot.eulerAngles(0,1,2);

    Eigen::Vector3d current_origin = RigidBodyDynamics::CalcBodyToBaseCoordinates(_rbdl_model,
                                          _zeros,
                                          _floating_base_link_id,
                                          Eigen::Vector3d(0,0,0) );

    _q.head(3) = _tmp_vector3d - _fb_origin_offset;

    chain("virtual_chain").setJointPosition(_q.head<6>());
    return true;
}

void XBot::ModelInterfaceRBDL::getCOMVelocity(KDL::Vector& velocity) const
{
    double mass;
    RigidBodyDynamics::Utils::CalcCenterOfMass(_rbdl_model, _q, _qdot, mass, _tmp_vector3d, &_tmp_vector3d_1, nullptr, true);
    tf::vectorEigenToKDL(_tmp_vector3d_1, velocity);
}

void XBot::ModelInterfaceRBDL::getGravity(KDL::Vector& gravity) const
{
    tf::vectorEigenToKDL(_rbdl_model.gravity, gravity);
}

bool XBot::ModelInterfaceRBDL::getAccelerationTwist(const std::string& link_name, KDL::Twist& acceleration) const
{
    int body_id = linkId(link_name);
    if( body_id == -1 ){
        Logger::error() << "in " << __func__ << ": link " << link_name << " not defined in RBDL model!" << Logger::endl();
        return false;
    }
    _tmp_vector3d.setZero();
    tf::twistEigenToKDL( _row_inversion*RigidBodyDynamics::CalcPointAcceleration6D(_rbdl_model, _q, _qdot, _qddot, body_id, _tmp_vector3d, true),
                        acceleration );
    return true;
}

bool XBot::ModelInterfaceRBDL::getRelativeAccelerationTwist(const std::string& link_name,
                                                            const std::string& base_link_name,
                                                            KDL::Twist& acceleration) const
{
    int body_id_a = linkId(base_link_name);
    if( body_id_a == -1 ){
        Logger::error() << "in " << __func__ << ": link " << base_link_name << " not defined in RBDL model!" << Logger::endl();
        return false;
    }

    int body_id_b = linkId(link_name);
    if( body_id_b == -1 ){
        Logger::error() << "in " << __func__ << ": link " << link_name << " not defined in RBDL model!" << Logger::endl();
        return false;
    }


    //1) Get Rotation
    Eigen::Affine3d Tbase;
    ModelInterface::getPose(base_link_name, Tbase);
    Eigen::Affine3d Tlink;
    ModelInterface::getPose(link_name, Tlink);

    //2) Get Accelerations
    Eigen::Vector6d abase, alink;
    ModelInterface::getAccelerationTwist(base_link_name, abase);
    ModelInterface::getAccelerationTwist(link_name, alink);

    //3) Get Velocities
    Eigen::Vector6d vbase, vlink;
    ModelInterface::getVelocityTwist(base_link_name, vbase);
    ModelInterface::getVelocityTwist(link_name, vlink);


    //4) Final result
    Eigen::Vector3d r = Tlink.translation() - Tbase.translation();
    Eigen::Vector6d a; a.setZero(6);
    a.head(3) = Tbase.linear().inverse()*(alink.head(3) - abase.head(3)
                                          - abase.tail<3>().cross(r)
                                          - 2.*vbase.tail<3>().cross((vlink-vbase).head<3>())
                                          + vbase.tail<3>().cross(vbase.tail<3>().cross(r)) );
    a.tail(3) = Tbase.linear().inverse()*((alink.tail(3)-abase.tail(3)) - (vbase.tail<3>().cross(vlink.tail<3>())));


    acceleration.vel[0] = a[0];
    acceleration.vel[1] = a[1];
    acceleration.vel[2] = a[2];
    acceleration.rot[0] = a[3];
    acceleration.rot[1] = a[4];
    acceleration.rot[2] = a[5];

    return true;
}

bool XBot::ModelInterfaceRBDL::computeRelativeJdotQdot(const std::string& target_link_name,
                                                       const std::string& base_link_name,
                                                       KDL::Twist& jdotqdot) const
{
    int body_id_a = linkId(base_link_name);
    if( body_id_a == -1 ){
        Logger::error() << "in " << __func__ << ": link " << base_link_name << " not defined in RBDL model!" << Logger::endl();
        return false;
    }

    int body_id_b = linkId(target_link_name);
    if( body_id_b == -1 ){
        Logger::error() << "in " << __func__ << ": link " << target_link_name << " not defined in RBDL model!" << Logger::endl();
        return false;
    }

    //1) Get Rotation
    Eigen::Affine3d Tbase;
    ModelInterface::getPose(base_link_name, Tbase);
    Eigen::Affine3d Tlink;
    ModelInterface::getPose(target_link_name, Tlink);

    //2) GetJDotQdot
    Eigen::Vector6d JdQdbase;
    Eigen::Vector3d zero; zero.setZero();
    ModelInterface::computeJdotQdot(base_link_name, zero, JdQdbase);
    Eigen::Vector6d JdQdlink;
    ModelInterface::computeJdotQdot(target_link_name, zero, JdQdlink);

    //3) Get Velocities
    Eigen::Vector6d vbase, vlink;
    ModelInterface::getVelocityTwist(base_link_name, vbase);
    ModelInterface::getVelocityTwist(target_link_name, vlink);



    //4) Final result
    Eigen::Vector3d r = Tlink.translation() - Tbase.translation();
    Eigen::Vector6d JdotQdot; JdotQdot.setZero(6);
    JdotQdot.head(3) = Tbase.linear().inverse()*(JdQdlink.head(3) - JdQdbase.head(3)
                                                 - JdQdbase.tail<3>().cross(r)
                                          - 2.*vbase.tail<3>().cross((vlink-vbase).head<3>())
                                          + vbase.tail<3>().cross(vbase.tail<3>().cross(r)) );
    JdotQdot.tail(3) = Tbase.linear().inverse()*((JdQdlink.tail(3)-JdQdbase.tail(3)) - (vbase.tail<3>().cross(vlink.tail<3>())));


    jdotqdot.vel[0] = JdotQdot[0];
    jdotqdot.vel[1] = JdotQdot[1];
    jdotqdot.vel[2] = JdotQdot[2];
    jdotqdot.rot[0] = JdotQdot[3];
    jdotqdot.rot[1] = JdotQdot[4];
    jdotqdot.rot[2] = JdotQdot[5];

    return true;
}

bool XBot::ModelInterfaceRBDL::getVelocityTwist(const std::string& link_name, KDL::Twist& velocity) const
{
    int body_id = linkId(link_name);
    if( body_id == -1 ){
        Logger::error() << "in " << __func__ << ": link " << link_name << " not defined in RBDL model!" << Logger::endl();
        return false;
    }

    _tmp_vector3d.setZero();
    tf::twistEigenToKDL(_row_inversion*RigidBodyDynamics::CalcPointVelocity6D(_rbdl_model, 
                                                                              _q, 
                                                                              _qdot, 
                                                                              body_id,
                                                                              _tmp_vector3d, 
                                                                              false), 
                        velocity);
    return true;
}


void XBot::ModelInterfaceRBDL::setGravity(const KDL::Vector& gravity)
{
    tf::vectorKDLToEigen(gravity, _rbdl_model.gravity);
}

void XBot::ModelInterfaceRBDL::computeGravityCompensation(Eigen::VectorXd& g) const
{
    g.resize(_ndof);

    RigidBodyDynamics::NonlinearEffects(_rbdl_model,
                                        _q,
                                        _zeros,
                                        g);
    
    RigidBodyDynamics::NonlinearEffects(_rbdl_model,
                                        _q,
                                        _qdot,
                                        _jtmp);
}

void XBot::ModelInterfaceRBDL::computeInverseDynamics(Eigen::VectorXd& tau) const
{
    tau.setZero(_ndof);
    RigidBodyDynamics::InverseDynamics(_rbdl_model, _q, _qdot, _qddot, tau);
}

void XBot::ModelInterfaceRBDL::computeNonlinearTerm(Eigen::VectorXd& n) const
{
    n.setZero(_ndof);

    RigidBodyDynamics::NonlinearEffects(_rbdl_model,
                                        _q,
                                        _qdot,
                                        n );
}

bool XBot::ModelInterfaceRBDL::computeJdotQdot(const std::string& link_name,
                                               const KDL::Vector& point,
                                               KDL::Twist& jdotqdot) const
{
    int body_id = linkId(link_name);
    if( body_id == -1 ){
        Logger::error() << "in " << __func__ << ": link " << link_name << " not defined in RBDL model!" << Logger::endl();
        return false;
    }

    tf::vectorKDLToEigen(point, _tmp_vector3d);
    tf::twistEigenToKDL( _row_inversion*RigidBodyDynamics::CalcPointAcceleration6D(_rbdl_model, _q, _qdot, _qddot*0, body_id, _tmp_vector3d, true),
                          jdotqdot );
    return true;

}

void XBot::ModelInterfaceRBDL::getCOMAcceleration(KDL::Vector& acceleration) const
{
    double mass = 0;
    int body_id = 0;
    _tmp_vector3d.setZero();

    RigidBodyDynamics::UpdateKinematics(_rbdl_model, _q, _qdot, _qddot);

    for( const RigidBodyDynamics::Body& body : _rbdl_model.mBodies ){

        _tmp_vector3d += body.mMass*RigidBodyDynamics::CalcPointAcceleration(_rbdl_model,
                                                                            _q, _qdot, _qddot,
                                                                            body_id++,
                                                                            body.mCenterOfMass,
                                                                            false);

        mass += body.mMass;
    }

    _tmp_vector3d /= mass;

    tf::vectorEigenToKDL(_tmp_vector3d, acceleration);
}

void XBot::ModelInterfaceRBDL::getInertiaMatrix(Eigen::MatrixXd& M) const
{
    M.setZero(_ndof, _ndof);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(_rbdl_model, _q, M, false);
}

void XBot::ModelInterfaceRBDL::getInertiaInverseTimesVector(const Eigen::VectorXd& vec, Eigen::VectorXd& minv_vec) const
{
    minv_vec.setZero(getJointNum());

    ///TODO: REMOVE THIS SHIT!
    RigidBodyDynamics::UpdateKinematics(_rbdl_model, _q, _qdot*0.0, _qddot);

    RigidBodyDynamics::CalcMInvTimesTau(_rbdl_model, _q, vec, minv_vec, true);

    ///TODO: REMOVE THIS SHIT!
    RigidBodyDynamics::UpdateKinematics(_rbdl_model, _q, _qdot, _qddot);
}

bool XBot::ModelInterfaceRBDL::getPointAcceleration(const std::string& link_name,
                                                    const KDL::Vector& point,
                                                    KDL::Vector& acceleration) const
{
    int body_id = linkId(link_name);
    if( body_id == -1 ){
        Logger::error() << "ERROR in " << __func__ << ": link " << link_name << " not defined in RBDL model!" << Logger::endl();
        return false;
    }

    tf::vectorKDLToEigen(point, _tmp_vector3d);
    tf::vectorEigenToKDL( RigidBodyDynamics::CalcPointAcceleration(_rbdl_model, _q, _qdot, _qddot, body_id, _tmp_vector3d, true),
                          acceleration );
    return true;
}

int XBot::ModelInterfaceRBDL::getLinkID(const std::string &link_name) const
{
    unsigned int id = _rbdl_model.GetBodyId(link_name.c_str());
    if(id == std::numeric_limits<unsigned int>::max())
        return -1;
    return id;
}


void XBot::ModelInterfaceRBDL::getCentroidalMomentum(Eigen::Vector6d& centroidal_momentum) const
{
    double mass;
    RigidBodyDynamics::Math::Vector3d tmp_2;
    RigidBodyDynamics::Utils::CalcCenterOfMass(_rbdl_model, _q, _qdot, mass, tmp_2, &_tmp_vector3d, &_tmp_vector3d_1, false);
    centroidal_momentum.head<3>() = _tmp_vector3d*mass;
    centroidal_momentum.tail<3>() = _tmp_vector3d_1;
}

double XBot::ModelInterfaceRBDL::getMass() const
{
    return _mass;
}

bool XBot::ModelInterfaceRBDL::getFloatingBaseLink(std::string& floating_base_link) const
{
    if(!isFloatingBase()){
        return false;
        Logger::error() << "in " << __func__ << ": model is not floating base!" << Logger::endl();
    }

    floating_base_link = _floating_base_link;
    return true;
}


