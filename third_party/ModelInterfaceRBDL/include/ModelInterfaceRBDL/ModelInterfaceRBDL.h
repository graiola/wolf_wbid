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

#ifndef XBOT_MODELINTERFACE_RBDL_H
#define XBOT_MODELINTERFACE_RBDL_H

#include <XBotInterface/ModelInterface.h>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>


namespace XBot
{

class ModelInterfaceRBDL : public ModelInterface
{

public:

    virtual void getCOM(KDL::Vector& com_position) const;

    virtual void getCOMJacobian(KDL::Jacobian& J) const;

    virtual void getCOMVelocity(KDL::Vector& velocity) const;

    virtual void getGravity(KDL::Vector& gravity) const;

    virtual void getModelOrderedJoints(std::vector< std::string >& joint_name) const;

    virtual bool getJacobian(const std::string& link_name, const KDL::Vector& reference_point, KDL::Jacobian& J) const;

    virtual bool getPose(const std::string& source_frame, KDL::Frame& pose) const;

    virtual bool getAccelerationTwist(const std::string& link_name, KDL::Twist& acceleration) const;

    virtual bool getRelativeAccelerationTwist(const std::string& link_name, const std::string& base_link_name,
                                              KDL::Twist& acceleration) const;

    virtual bool getVelocityTwist(const std::string& link_name, KDL::Twist& velocity) const;

    virtual bool setFloatingBasePose(const KDL::Frame& floating_base_pose);

    virtual bool setFloatingBaseTwist(const KDL::Twist& floating_base_twist);

    virtual void setGravity(const KDL::Vector& gravity);

    virtual void computeGravityCompensation(Eigen::VectorXd& g) const;

    virtual void getCentroidalMomentum(Eigen::Vector6d& centroidal_momentum) const;

    virtual void computeInverseDynamics(Eigen::VectorXd& tau) const;

    virtual void computeNonlinearTerm(Eigen::VectorXd& n) const;

    virtual bool computeJdotQdot(const std::string& link_name, const KDL::Vector& point, KDL::Twist& jdotqdot) const;

    virtual bool computeRelativeJdotQdot(const std::string& target_link_name,
                                         const std::string& base_link_name,
                                         KDL::Twist& jdotqdot) const;

    virtual void getCOMAcceleration(KDL::Vector& acceleration) const;

    virtual void getInertiaMatrix(Eigen::MatrixXd& M) const;

    virtual void getInertiaInverseTimesVector(const Eigen::VectorXd& vec, Eigen::VectorXd& minv_vec) const;

    virtual bool getPointAcceleration(const std::string& link_name,
                                      const KDL::Vector& point,
                                      KDL::Vector& acceleration) const;

    virtual bool update_internal(bool update_position,
                        bool update_velocity,
                        bool update_desired_acceleration);

    virtual int getLinkID(const std::string &link_name) const;

    virtual double getMass() const;

    virtual bool getFloatingBaseLink(std::string& floating_base_link) const;





protected:


    virtual bool init_model(const ConfigOptions& cfg) final;

    int linkId(const std::string& link_name) const;

    mutable RigidBodyDynamics::Model _rbdl_model;

private:

    Eigen::VectorXd _q, _qdot, _qddot, _tau;
    Eigen::Vector3d _fb_origin_offset;
    int _ndof;
    Eigen::VectorXd _zeros;
    mutable Eigen::VectorXd _jtmp;

    std::string _floating_base_link;
    int _floating_base_link_id;

    double _mass;

    std::vector<std::string> _model_ordered_joint_names;

    mutable RigidBodyDynamics::Math::Vector3d _tmp_vector3d, _tmp_vector3d_1;
    mutable RigidBodyDynamics::Math::Matrix3d _tmp_matrix3d;
    mutable RigidBodyDynamics::Math::MatrixNd _tmp_jacobian3, _tmp_jacobian6;
    mutable RigidBodyDynamics::Math::VectorNd _tmp_jstate;
    mutable KDL::Frame _tmp_kdl_frame;

    Eigen::Matrix<double, 6, 6> _row_inversion;



    int jointModelId(const std::string& joint_name) const;


};


}
#endif
