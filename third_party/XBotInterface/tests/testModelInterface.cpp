#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/Utils.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <chrono>
#include <time.h>
#include <numeric>

class testModelInterface : public testing::Test {

protected:

virtual void SetUp(){


    path_to_cfg = XBOT_TESTS_DIR "configs/centauro/configs/config_centauro_upperbody.yaml";

    path_to_full_body_cfg = XBOT_TESTS_DIR "configs/centauro/configs/config_centauro_full_body.yaml";

    std::cout << "PATH TO CFG: " << path_to_cfg << std::endl;

    model_ptr = XBot::ModelInterface::getModel(path_to_cfg);
    fb_model_ptr = XBot::ModelInterface::getModel(path_to_full_body_cfg);

}

virtual void TearDown(){}

XBot::ModelInterface::Ptr model_ptr, fb_model_ptr;
std::string path_to_cfg, path_to_full_body_cfg;

};

inline void computeCartesianError(const Eigen::Affine3d& ref, const Eigen::Affine3d& actual, Eigen::VectorXd& error)
{
    error.resize(6);

    Eigen::Quaterniond q(actual.linear()), q_d(ref.linear());
    Eigen::Vector3d orientation_error = q.w()*q_d.vec() - q_d.w()*q.vec() - q_d.vec().cross(q.vec());
    Eigen::Vector3d position_error = ref.translation() - actual.translation();

    error << position_error, orientation_error;
}

TEST_F(testModelInterface, checkJdotQdotImpl)
{
    Eigen::Vector6d JdotQdot_base, JdotQdot_rbdl;
    JdotQdot_base.setZero(6);
    JdotQdot_rbdl.setZero(6);

    KDL::Twist JdotQdot_base_KDL;
    KDL::Vector zero; zero.Zero();

    XBot::ModelInterface& model = *fb_model_ptr;
    Eigen::VectorXd q, qdot;


    std::vector<double> base_time;
    std::vector<double> rbdl_time;

    std::string link = "arm1_7";

    for(unsigned int i = 0; i < 1000; ++i)
    {
        q.setRandom(model.getJointNum());
        qdot.setRandom(model.getJointNum());
        model.setJointPosition(q);
        model.setJointVelocity(qdot);
        model.update();

        auto tic = std::chrono::high_resolution_clock::now();
        model.computeJdotQdot(link, Eigen::Vector3d::Zero(), JdotQdot_rbdl);
        auto toc = std::chrono::high_resolution_clock::now();
        rbdl_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(toc-tic).count());

    std::cout<<std::endl;

        tic = std::chrono::high_resolution_clock::now();
        model.ModelInterface::computeJdotQdot(link, zero, JdotQdot_base_KDL);
        toc = std::chrono::high_resolution_clock::now();
        base_time.push_back(std::chrono::duration_cast<std::chrono::microseconds>(toc-tic).count());

        tf::twistKDLToEigen(JdotQdot_base_KDL, JdotQdot_base);
        EXPECT_NEAR((JdotQdot_base - JdotQdot_rbdl).norm(), 0, 1e-6);

        std::cout<<std::endl;
        std::cout<<std::endl;
    }

    std::cout<<"time base class [us]: "<<std::accumulate(base_time.begin(), base_time.end(), 0)/base_time.size()<<std::endl;
    std::cout<<"time rbdl class [us]: "<<std::accumulate(rbdl_time.begin(), rbdl_time.end(), 0)/rbdl_time.size()<<std::endl;

}


TEST_F(testModelInterface, checkBasicIK){

    XBot::ModelInterface& model = *model_ptr;

    std::string end_effector_name = model.arm(0).getTipLinkName();

    Eigen::VectorXd q_home;
    model.getRobotState("home", q_home);
    model.setJointPosition(q_home);
    model.update();

    std::cout << "HOMING: \n" << q_home << std::endl;



    Eigen::Affine3d desired_pose, actual_pose, initial_pose;
    Eigen::VectorXd cartesian_error(6), xdot(6), qdot, q;
    Eigen::MatrixXd J;


    model.getPose(end_effector_name, initial_pose);
    model.getJointPosition(q);

    int max_iter = 100;
    int iter = 0;
    double dt = 0.01;

    while( iter < max_iter){

        // Compute desired end-effector pose
        desired_pose.translation() = initial_pose.translation() + Eigen::Vector3d(0,0,1)*0.2*(iter/double(max_iter));
        desired_pose.linear() = initial_pose.linear();

        std::cout << "DESIRED POSE\n" << "Position:\n" << desired_pose.translation() << "\nOrientation:\n" << desired_pose.linear() << std::endl;

        // Compute actual pose
        model.getPose(end_effector_name, actual_pose);

        std::cout << "ACTUAL POSE\n" << "Position:\n" << actual_pose.translation() << "\nOrientation:\n" << actual_pose.linear() << std::endl;


        // Compute cartesian error and rotate it to world frame
        computeCartesianError(desired_pose, actual_pose, cartesian_error);
        std::cout << "ERROR: \n" << cartesian_error << std::endl;

        // Compute end-effector velocity
        xdot = 100*cartesian_error;

        // Compute jacobian
        model.getJacobian(end_effector_name, J);

        // Set columns to zero except for arm(0)
        for( const std::string& chain_name : model.getChainNames() ){
            if( chain_name != model.arm(0).getChainName() ){

                std::vector<int> indices_to_be_disabled;
                model.getDofIndex(chain_name, indices_to_be_disabled);

                for(int idx : indices_to_be_disabled){
                    J.col(idx).setZero();
                }
            }
        }

        std::cout << "JACOBIAN:\n" << J << std::endl;

        qdot = J.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(xdot);

        // Integration step
        q += qdot*dt;

        // Update model state
        model.setJointPosition(q);
        model.update();

        iter++;

    }

    std::cout << cartesian_error << std::endl;
    EXPECT_NEAR( cartesian_error.norm(), 0, 0.01 );

}

TEST_F(testModelInterface, checkNumericalJacobian){

    XBot::ModelInterface& model = *model_ptr;

    Eigen::VectorXd q, q1, q2;
    q.setRandom(model.getJointNum());
    double step_size = 0.0001;


    Eigen::MatrixXd J, Jhat;
    Eigen::Affine3d T, T1, T2, dT;
    Eigen::Vector3d p, p1, p2, dPhi;
    Eigen::Matrix3d R, R1, R2, S;

    for( const std::string& j : model.getEnabledJointNames() ){

        std::string link_name = model.getUrdf().getJoint(j)->child_link_name;

        model.setJointPosition(q);
        model.update();

        model.getPose(link_name, T);
        p = T.translation();
        R = T.linear();

        model.getJacobian(link_name, J);
        Jhat = J;

        for( int i = 1; i < model.getJointNum(); i++ ){

            q2 = q;
            q2(i) += step_size/2;

            q1 = q;
            q1(i) -= step_size/2;

            model.setJointPosition(q1);
            model.update();
            model.getPose(link_name, T1);
            p1 = T1.translation();
            R1 = T1.linear();

            model.setJointPosition(q2);
            model.update();
            model.getPose(link_name, T2);
            p2 = T2.translation();
            R2 = T2.linear();

            Jhat.col(i).head(3) = (p2-p1)/step_size;
            S = (R2-R1)*R.transpose();
            Jhat(3,i) = S(2,1)/step_size;
            Jhat(4,i) = S(0,2)/step_size;
            Jhat(5,i) = S(1,0)/step_size;


        }

        std::cout << "Link: " << link_name << ",  (J-Jhat).norm()/J.size() = " << (J-Jhat).norm()/J.size() << std::endl;

        EXPECT_NEAR( (J-Jhat).norm()/J.size() , 0, 0.001 );

    }

}

TEST_F(testModelInterface, checkPoseConsistency){

    XBot::ModelInterface& model = *model_ptr;
    Eigen::VectorXd q;
    q.setRandom(model.getJointNum());
    model.setJointPosition(q);
    model.update();

    std::vector<std::string> link_names;

    for( const auto& j : model.getEnabledJointNames() ){
        link_names.push_back(model.getUrdf().getJoint(j)->child_link_name);
    }

    Eigen::Affine3d b_T_a, c_T_a, c_T_b;

    for( const std::string& link_a : link_names ){
        for( const std::string& link_b : link_names ){
            for( const std::string& link_c : link_names ){


                model.getPose(link_a, link_b, b_T_a);
                model.getPose(link_b, link_c, c_T_b);
                model.getPose(link_a, link_c, c_T_a);

                EXPECT_NEAR( ((c_T_b*b_T_a).translation() - c_T_a.translation()).norm()/3, 0.0, 0.0001 );
                EXPECT_NEAR( ((c_T_b*b_T_a).linear() - c_T_a.linear()).norm()/9, 0.0, 0.0001 );


            }

            model.getPose(link_a, link_b, b_T_a);
            model.getPose(link_b, c_T_b);
            model.getPose(link_a, c_T_a);

            EXPECT_NEAR( ((c_T_b*b_T_a).translation() - c_T_a.translation()).norm()/3, 0.0, 0.0001 );
            EXPECT_NEAR( ((c_T_b*b_T_a).linear() - c_T_a.linear()).norm()/9, 0.0, 0.0001 );

        }
    }



}

TEST_F(testModelInterface, checkJacobianMask){

    Eigen::MatrixXd J;


    auto chains = model_ptr->getChainNames();

    J.setRandom(6, model_ptr->getJointNum());

    for( int iter = 0; iter < 100; iter++){

        int random_chain = rand()%chains.size();

        model_ptr->maskJacobian(chains[random_chain], J);

        for(int i = 0; i < model_ptr->chain(chains[random_chain]).getJointNum(); i++){
            EXPECT_EQ( J.col(model_ptr->getDofIndex(model_ptr->chain(chains[random_chain]).getJointName(i))).norm(), 0.0 );
        }

    }



}


TEST_F(testModelInterface, checkTwistVsPose){
    
    XBot::ModelInterface& model = *model_ptr;


    std::vector<urdf::LinkSharedPtr> link_list;
    model.getUrdf().getLinks(link_list);    
    
    Eigen::Vector6d vrel;
    Eigen::MatrixXd Jrel;
    
    
    for(int i = 0; i < 100; i++)
    {
    
    Eigen::VectorXd q, q1, q2, qdot;
    q.setRandom(model.getJointNum());
    qdot.setRandom(model.getJointNum());
    
    for(urdf::LinkSharedPtr link_a : link_list)
    {
        
        static int iter = 0;
        std::cout << iter++ << std::endl;
        
        model.setJointPosition(q);
        model.setJointVelocity(qdot);
        model.update();
        Eigen::Affine3d T;
        model.getPose(link_a->name, T);
        
        ASSERT_TRUE(model.getVelocityTwist(link_a->name, vrel));
        
        const double dt = 0.0001;
        q1 = q - qdot * dt / 2.0;
        q2 = q + qdot * dt / 2.0;
        
        model.setJointPosition(q1);
        model.update();
        Eigen::Affine3d T1;
        model.getPose(link_a->name, T1);
        
        model.setJointPosition(q2);
        model.update();
        Eigen::Affine3d T2;
        model.getPose(link_a->name, T2);
        
        Eigen::Vector3d v_lin_diff = (T2.translation() - T1.translation())/dt;
        Eigen::Matrix3d Rdot = (T2.linear() - T1.linear()) / dt;
        Eigen::Matrix3d S = Rdot * T.linear().transpose();
        Eigen::Vector3d v_ang_diff(S(2,1), S(0,2), S(1,0));
        
        EXPECT_NEAR( (v_lin_diff - vrel.head<3>()).norm(), 0.0, 1e-6 );
        EXPECT_NEAR( (v_ang_diff - vrel.tail<3>()).norm(), 0.0, 1e-6 );
        
    }
    
    }

    
}

TEST_F(testModelInterface, checkRelativeTwist){
    
    XBot::ModelInterface& model = *model_ptr;


    Eigen::VectorXd q, q1, q2, qdot;
    q.setRandom(model.getJointNum());
    qdot.setRandom(model.getJointNum());


    model.setJointPosition(q);
    model.setJointVelocity(qdot);
    model.update();
    
    std::vector<urdf::LinkSharedPtr> link_list;
    model.getUrdf().getLinks(link_list);    
    
    Eigen::Vector6d vrel;
    Eigen::MatrixXd Jrel;
    
    for(urdf::LinkSharedPtr link_a : link_list)
    {
        for(urdf::LinkSharedPtr link_b : link_list)
        {
            static int iter = 0;
            std::cout << iter++ << std::endl;
            
            model.setJointPosition(q);
            model.setJointVelocity(qdot);
            model.update();
            Eigen::Affine3d T;
            model.getPose(link_a->name, link_b->name, T);
            
            ASSERT_TRUE(model.getRelativeVelocityTwist(link_a->name, link_b->name, vrel));
//             ASSERT_TRUE(model.getRelativeJacobian(link_a->name, link_b->name, Jrel));
//             vrel = Jrel * qdot;
            
            const double dt = 0.0001;
            q1 = q - qdot * dt / 2.0;
            q2 = q + qdot * dt / 2.0;
            
            model.setJointPosition(q1);
            model.update();
            Eigen::Affine3d T1;
            model.getPose(link_a->name, link_b->name, T1);
            
            model.setJointPosition(q2);
            model.update();
            Eigen::Affine3d T2;
            model.getPose(link_a->name, link_b->name, T2);
            
            Eigen::Vector3d v_lin_diff = (T2.translation() - T1.translation())/dt;
            Eigen::Matrix3d Rdot = (T2.linear() - T1.linear()) / dt;
            Eigen::Matrix3d S = Rdot * T.linear().transpose();
            Eigen::Vector3d v_ang_diff(S(2,1), S(0,2), S(1,0));
            
            EXPECT_NEAR( (v_lin_diff - vrel.head<3>()).norm(), 0.0, 1e-6 );
            EXPECT_NEAR( (v_ang_diff - vrel.tail<3>()).norm(), 0.0, 1e-6 );
        }
        
    }

    
}

TEST_F(testModelInterface, checkRelativeTwistVsJacobian){

    XBot::ModelInterface& model = *model_ptr;


    Eigen::VectorXd q, qdot;
    q.setRandom(model.getJointNum());
    qdot.setRandom(model.getJointNum());


    model.setJointPosition(q);
    model.setJointVelocity(qdot);
    model.update();
       
    
    std::vector<urdf::LinkSharedPtr> link_list;
    model.getUrdf().getLinks(link_list);    
    
    KDL::Jacobian Jrel;
    Eigen::Vector6d vrel;
    
    for(urdf::LinkSharedPtr link_a : link_list)
    {
        for(urdf::LinkSharedPtr link_b : link_list)
        {
            static int iter = 0;
            std::cout << iter++ << std::endl;
            
            ASSERT_TRUE(model.getRelativeVelocityTwist(link_a->name, link_b->name, vrel));
            ASSERT_TRUE(model.getRelativeJacobian(link_a->name, link_b->name, Jrel));
            
            EXPECT_NEAR( (vrel - Jrel.data*qdot).norm(), 0.0, 1e-6 );
        }
        
    }

}

TEST_F( testModelInterface, checkCOM ){

    XBot::ModelInterface& model = *model_ptr;

    for(int iter=0; iter < 100; iter++){

    Eigen::VectorXd q, q1, q2, qdot;
    q.setRandom(model.getJointNum());
    qdot.setRandom(model.getJointNum());


    model.setJointPosition(q);
    model.setJointVelocity(qdot);
    model.update();

    Eigen::MatrixXd Jcom, Jcom_hat;
    Eigen::Vector3d com, com1, com2, dcom;
    model.getCOMJacobian(Jcom);
    model.getCOM(com);
    model.getCOMVelocity(dcom);

    EXPECT_EQ( Jcom.rows(), 3 );
    EXPECT_EQ( Jcom.cols(), model.getJointNum() );


    Eigen::Vector3d dcom1;
    dcom1 = Jcom*qdot;

    EXPECT_NEAR( (dcom-dcom1).norm(), 0, 0.0001 );

    double step_size = 0.000001;
    Jcom_hat.resize(Jcom.rows(), Jcom.cols());

    for(int i = 0; i < model.getJointNum(); i++){

        q1 = q;
        q2 = q;
        q2(i) += step_size/2;
        q1(i) -= step_size/2;

        model.setJointPosition(q2);
        model.update();
        model.getCOM(com2);

        model.setJointPosition(q1);
        model.update();
        model.getCOM(com1);


        Jcom_hat.col(i) = (com2-com1)/step_size;


    }


    EXPECT_NEAR( (Jcom-Jcom_hat).norm()/Jcom.size(), 0, 0.00001 );
    EXPECT_NEAR( (dcom-Jcom_hat*qdot).norm(), 0, 0.0001);

    }



}


TEST_F( testModelInterface, checkCOM2 ){

    XBot::ModelInterface& model = *fb_model_ptr;

    for(int iter=0; iter < 100; iter++){

        Eigen::VectorXd q, q1, q2, qdot;
        q.setRandom(model.getJointNum());
        qdot.setRandom(model.getJointNum());


        model.setJointPosition(q);
        model.setJointVelocity(qdot);
        model.update();

        Eigen::MatrixXd Jcom, Jcom_hat, Jcom2;
        Eigen::Vector3d com, com1, com2, dcom, jdotcomqdot;
        model.getCOMJacobian(Jcom);
        model.getCOMJacobian(Jcom2, jdotcomqdot);
        model.getCOM(com);
        model.getCOMVelocity(dcom);
        
        std::cout << "Jcom2:\n" << Jcom << std::endl;
        std::cout << "Jcom:\n" << Jcom2 << std::endl;

        EXPECT_EQ( Jcom.rows(), 3 );
        EXPECT_EQ( Jcom.cols(), model.getJointNum() );
        EXPECT_NEAR( (Jcom-Jcom2).norm(), 0, 0.00001 );


        Eigen::Vector3d dcom1;
        dcom1 = Jcom*qdot;
        
        std::cout << "dCom/dt: " << dcom.transpose() << "   " << dcom1.transpose() << std::endl;

        EXPECT_NEAR( (dcom-dcom1).norm(), 0, 0.0001 );

        double step_size = 0.000001;
        Jcom_hat.resize(Jcom.rows(), Jcom.cols());

        for(int i = 0; i < model.getJointNum(); i++){

            q1 = q;
            q2 = q;
            q2(i) += step_size/2;
            q1(i) -= step_size/2;

            model.setJointPosition(q2);
            model.update();
            model.getCOM(com2);

            model.setJointPosition(q1);
            model.update();
            model.getCOM(com1);


            Jcom_hat.col(i) = (com2-com1)/step_size;


        }


        EXPECT_NEAR( (Jcom-Jcom_hat).norm()/Jcom.size(), 0, 0.00001 );
        EXPECT_NEAR( (dcom-Jcom_hat*qdot).norm(), 0, 0.0001);

    }



}


Eigen::Vector6d getPartialDerivative(const Eigen::MatrixXd& J, int joint_idx, int col_idx)
{
    using namespace XBot::Utils;
    
    int j = joint_idx;
    int i = col_idx;

    Eigen::Vector6d jac_j_ = J.col(j);
    Eigen::Vector6d jac_i_ = J.col(i);

    Eigen::Vector6d t_djdq_;
    t_djdq_.setZero();

    if(j < i)
    {
        // P_{\Delta}({}_{bs}J^{j})  ref (20)
        t_djdq_.head<3>() = skewSymmetricMatrix(jac_j_.tail<3>()) * jac_i_.head<3>();
        t_djdq_.tail<3>() = skewSymmetricMatrix(jac_j_.tail<3>()) * jac_i_.tail<3>();
    }else if(j > i)
    {
        // M_{\Delta}({}_{bs}J^{j})  ref (23)
        t_djdq_.tail<3>().setZero();
        t_djdq_.head<3>() = -skewSymmetricMatrix(jac_j_.head<3>()) * jac_i_.tail<3>();
    }else if(j == i)
    {
        // ref (40)
        t_djdq_.tail<3>().setZero();
        t_djdq_.head<3>() = skewSymmetricMatrix(jac_i_.tail<3>()) * jac_i_.head<3>();
    }
    
    return t_djdq_;
}

void computeJdot(const Eigen::MatrixXd& J, const Eigen::VectorXd& qdot, Eigen::MatrixXd& Jdot)
{
    Jdot = J*0;
    
    int k = 0;
    
    Eigen::Vector6d jac_dot_k;
    jac_dot_k.setZero();
    
    for(int i = 0; i < J.cols(); i++)
    {

        for(int j = 0; j < J.cols(); j++)
        {
            // Column J is the sum of all partial derivatives  ref (41)
            jac_dot_k += getPartialDerivative(J, j, k) * qdot(j);
        }
        
        Jdot.col(k++) = jac_dot_k;
        
        jac_dot_k.setZero();
    }
    
}


TEST_F( testModelInterface, checkJdotQdot ){

    XBot::ModelInterface& model = *fb_model_ptr;
    std::string ee_name = model.arm(0).getTipLinkName();

    Eigen::VectorXd q, qdot;
    Eigen::Matrix<double,6,1> jdotqdot, jdotqdot_numerical, jdotqdot_ana;
    Eigen::MatrixXd J1, J2, Jdot, Jdot_analytic;
    
    std::vector<urdf::LinkSharedPtr> links;
    fb_model_ptr->getUrdf().getLinks(links);
    
    for(int k = 0; k < links.size(); k++){
        
        ee_name = links[k]->name;

        for(int i = 0; i < 100; i++){

            std::cout << "link " << ee_name << "   i = " << i << std::endl;
            
            q.setRandom(model.getJointNum());
            qdot.setRandom(q.size());

            model.setJointPosition(q);
            model.setJointVelocity(qdot);
            model.update();
            
            Eigen::Vector3d ref_point = Eigen::Vector3d::Zero();

            model.computeJdotQdot(ee_name, ref_point, jdotqdot);
            

            double dt = 1e-6;

            model.setJointPosition(q + 0.5*dt*qdot);
            model.update();
            model.getJacobian(ee_name, ref_point, J2);

            model.setJointPosition(q - 0.5*dt*qdot);
            model.update();
            model.getJacobian(ee_name, ref_point, J1);

            Jdot = (J2 - J1)/dt;
            computeJdot((J1+J2)/2, qdot, Jdot_analytic);
            
            jdotqdot_numerical = Jdot * qdot;
            jdotqdot_ana = Jdot_analytic * qdot;

            EXPECT_NEAR( (jdotqdot-jdotqdot_numerical).norm()/model.getJointNum(), 0.0,  1e-6 );
            EXPECT_NEAR( (jdotqdot-jdotqdot_ana).norm()/model.getJointNum(), 0.0,  1e-6 );
            EXPECT_NEAR( (jdotqdot_numerical-jdotqdot_ana).norm()/model.getJointNum(), 0.0,  1e-6 );
            EXPECT_NEAR( (Jdot-Jdot_analytic).norm(), 0, 0.0001 );

        }
        
    }

}


TEST_F( testModelInterface, checkXddot ){

    XBot::ModelInterface& model = *fb_model_ptr;
    std::string ee_name = model.arm(0).getTipLinkName();

    Eigen::VectorXd q, qdot, qddot;
    Eigen::Vector6d jdotqdot;
    Eigen::MatrixXd J, Jdot_analytic;
    
    
    std::vector<urdf::LinkSharedPtr> links;
    fb_model_ptr->getUrdf().getLinks(links);
    
    for(int k = 0; k < links.size(); k++){
        
        ee_name = links[k]->name;

        for(int i = 0; i < 100; i++){
            
            std::cout << "link " << ee_name << "   i = " << i << std::endl;
            

            q.setRandom(model.getJointNum());
            qdot.setRandom(q.size());
            qddot.setRandom(q.size());

            model.setJointPosition(q);
            model.setJointVelocity(qdot);
            model.setJointAcceleration(qddot);
            model.update();

            model.computeJdotQdot(ee_name, Eigen::Vector3d::Zero(), jdotqdot);

            model.getJacobian(ee_name, J);

            computeJdot(J, qdot, Jdot_analytic);
            
            Eigen::Vector6d xddot, xddot_comp, xddot_comp_analytic;
            
            model.getAccelerationTwist(ee_name, xddot);
            
            xddot_comp = J*qddot + jdotqdot;
            xddot_comp_analytic = J*qddot + Jdot_analytic*qdot;
            
            EXPECT_NEAR( (xddot-xddot_comp).norm(), 0, 0.00001 );
            EXPECT_NEAR( (xddot-xddot_comp_analytic).norm(), 0, 0.00001 );

        }
        
    }

}


TEST_F( testModelInterface, checkSetFloatingBasePose ){


    for(int i = 0; i < 100; i++){

        Eigen::AngleAxisd rot;

        rot.axis().setRandom();
        rot.axis() /= rot.axis().norm();

        Eigen::Matrix2d random;
        random.setRandom();
        rot.angle() = random(0);

        Eigen::Affine3d pose;
        pose.linear() = rot.toRotationMatrix();
        pose.translation().setRandom();

        fb_model_ptr->setFloatingBasePose(pose);
        fb_model_ptr->update();

        Eigen::Affine3d actual_pose;
        fb_model_ptr->getPose("pelvis", actual_pose);

        EXPECT_TRUE( pose.isApprox(actual_pose, 0.0001) );

        fb_model_ptr->getFloatingBasePose(actual_pose);
        EXPECT_TRUE( pose.isApprox(actual_pose, 0.0001) );

    }


}

TEST_F( testModelInterface, checkSetFloatingBasePoseTwist ){


    for(int i = 0; i < 100; i++){

        Eigen::Vector6d twist, actual_twist;
        Eigen::VectorXd q, qdot;
        q.setRandom(fb_model_ptr->getJointNum());
        qdot.setRandom(fb_model_ptr->getJointNum());

        twist.setRandom();

        fb_model_ptr->setJointPosition(q);
        fb_model_ptr->setJointVelocity(qdot);
        fb_model_ptr->update();
        fb_model_ptr->setFloatingBaseTwist(twist);
        fb_model_ptr->update();

        fb_model_ptr->getVelocityTwist("pelvis", actual_twist);

        EXPECT_NEAR( (twist-actual_twist).norm(), 0, 0.0001 );

        fb_model_ptr->getFloatingBaseTwist(actual_twist);

        EXPECT_NEAR( (twist-actual_twist).norm(), 0, 0.0001 );

    }


}


TEST_F( testModelInterface, checkLocalJacobian ){


    std::vector<urdf::LinkSharedPtr> links;
    fb_model_ptr->getUrdf().getLinks(links);

    for(int i = 0; i < links.size(); i++){

        std::string link_name = links[i]->name;

        Eigen::MatrixXd J1, J2;
        Eigen::VectorXd q;
        q.setRandom(fb_model_ptr->getJointNum());


        fb_model_ptr->setJointPosition(q);
        fb_model_ptr->update();

        ASSERT_TRUE(fb_model_ptr->getJacobian(link_name, J1));
        ASSERT_TRUE(fb_model_ptr->getJacobian(link_name, link_name, J2));

        Eigen::Matrix3d w_R_link;
        fb_model_ptr->getOrientation(link_name, w_R_link);
        Eigen::MatrixXd w_I_link(6,6);

        w_I_link << w_R_link, Eigen::Matrix3d::Zero(),
                    Eigen::Matrix3d::Zero(), w_R_link;

        J1 = w_I_link.transpose() * J1;



        EXPECT_NEAR( (J1 - J2).norm(), 0, 0.0001 );

    }


}

TEST_F( testModelInterface, checkCmm)
{
    for(int i = 0; i < 100; i++){

        Eigen::Vector6d cmom, actual_cmom;
        Eigen::VectorXd q, qdot;
        Eigen::MatrixXd cmm;
        
        q.setRandom(fb_model_ptr->getJointNum());
        qdot.setRandom(fb_model_ptr->getJointNum());

        fb_model_ptr->setJointPosition(q);
        fb_model_ptr->setJointVelocity(qdot);
        fb_model_ptr->update();


        fb_model_ptr->getCentroidalMomentum(actual_cmom);
        fb_model_ptr->getCentroidalMomentumMatrix(cmm);
        cmom = cmm * qdot;
        
        std::cout << cmom.transpose() << std::endl;
        std::cout << actual_cmom.transpose() << std::endl;

        EXPECT_NEAR( (cmom-actual_cmom).norm(), 0, 0.0001 );



    }


}

TEST_F( testModelInterface, checkCmmDotQdot ){


    for(int i = 0; i < 100; i++){

        Eigen::Vector6d cmom, actual_cmom, cmmdotqdot, cmmdotqdot_est;
        Eigen::VectorXd q, qdot;
        Eigen::MatrixXd cmm, cmmdot, cmm1;
        
        q.setRandom(fb_model_ptr->getJointNum());
        qdot.setRandom(fb_model_ptr->getJointNum());

        fb_model_ptr->setJointPosition(q);
        fb_model_ptr->setJointVelocity(qdot);
        fb_model_ptr->update();


        fb_model_ptr->getCentroidalMomentum(actual_cmom);
        fb_model_ptr->getCentroidalMomentumMatrix(cmm, cmmdotqdot);
        cmom = cmm * qdot;
        
        std::cout << cmom.transpose() << std::endl;
        std::cout << actual_cmom.transpose() << "\n------" << std::endl;

        EXPECT_NEAR( (cmom-actual_cmom).norm(), 0, 0.0001 );
        
        double dt = 1e-6;
        fb_model_ptr->setJointPosition(q + qdot*dt);
        fb_model_ptr->update();
        fb_model_ptr->getCentroidalMomentumMatrix(cmm1);
        
        cmmdot = (cmm1 - cmm)/dt;
        
        cmmdotqdot_est = cmmdot * qdot;
        
        std::cout << cmmdotqdot.transpose() << std::endl;
        std::cout << cmmdotqdot_est.transpose() << std::endl;
        
        EXPECT_NEAR( (cmmdotqdot-cmmdotqdot_est).norm(), 0, 0.0001 );
        




    }


}


TEST_F( testModelInterface, checkInverseInertiaMatrix)
{
    for(int i = 0; i < 100; i++){

        Eigen::Vector6d twist, actual_twist;
        Eigen::VectorXd q, qdot;
        q.setRandom(fb_model_ptr->getJointNum());
        qdot.setRandom(fb_model_ptr->getJointNum());

        fb_model_ptr->setJointPosition(q);
        fb_model_ptr->setJointVelocity(qdot);
        fb_model_ptr->update();

        Eigen::MatrixXd M(fb_model_ptr->getJointNum(), fb_model_ptr->getJointNum());
        fb_model_ptr->getInertiaMatrix(M);
        //std::cout<<"M: \n"<<M<<std::endl;



        Eigen::MatrixXd M_inverse = M.inverse();
        Eigen::VectorXd Minvq = M_inverse*(q);
//        std::cout<<"Minvq: \n"<<Minvq.transpose()<<std::endl;
        Eigen::VectorXd Minvq2;
        fb_model_ptr->getInertiaInverseTimesVector(q, Minvq2);
        //std::cout<<"M*Minvq2q: \n"<<((M*Minvq2)).transpose()<<std::endl;
        EXPECT_NEAR((Minvq - Minvq2).norm(), 0.0, 1e-7);




        Eigen::MatrixXd T(M.rows(), M.cols());
        T.setRandom(M.rows(), M.cols());

        Eigen::MatrixXd I = M_inverse*T;
        //std::cout<<"I: \n"<<I<<std::endl;
        Eigen::MatrixXd I2;
        fb_model_ptr->getInertiaInverseTimesMatrix(T, I2);
        //std::cout<<"I2: \n"<<I2<<std::endl;
        EXPECT_NEAR((I - I2).norm(), 0.0, 1e-7);



        Eigen::MatrixXd M_inverse2(fb_model_ptr->getJointNum(), fb_model_ptr->getJointNum());
        fb_model_ptr->getInertiaInverse(M_inverse2);
        //std::cout<<"M_inverse2: \n"<<M_inverse2<<std::endl;
        //std::cout<<"M2: \n"<<M_inverse2.inverse()<<std::endl;
        EXPECT_NEAR((M_inverse - M_inverse2).norm(), 0.0, 1e-7);
    }
}

TEST_F( testModelInterface, checkVelocityTwist ){
 
    for(int i = 0; i < 100; i++){
     
        Eigen::Vector6d twist, actual_twist;
        Eigen::VectorXd q, qdot;
        q.setRandom(fb_model_ptr->getJointNum());
        qdot.setRandom(fb_model_ptr->getJointNum());

        fb_model_ptr->setJointPosition(q);
        fb_model_ptr->setJointVelocity(qdot);
        fb_model_ptr->update();
        
        Eigen::MatrixXd J;
        
        std::vector<urdf::LinkSharedPtr> links;
        fb_model_ptr->getUrdf().getLinks(links);
        
        for(auto link: links){
            
            ASSERT_TRUE(fb_model_ptr->getVelocityTwist(link->name, twist));
            ASSERT_TRUE(fb_model_ptr->getJacobian(link->name, J));
            actual_twist = J*qdot;
            EXPECT_NEAR((twist-actual_twist).norm(), 0.0, 1e-6);
            
        }
        
        
    }
    
    
    
    for(int i = 0; i < 100; i++){
     
        Eigen::Vector6d twist, actual_twist;
        Eigen::VectorXd q, qdot;
        q.setRandom(model_ptr->getJointNum());
        qdot.setRandom(model_ptr->getJointNum());

        model_ptr->setJointPosition(q);
        model_ptr->setJointVelocity(qdot);
        model_ptr->update();
        
        Eigen::MatrixXd J;
        
        std::vector<urdf::LinkSharedPtr> links;
        model_ptr->getUrdf().getLinks(links);
        
        for(auto link: links){
            
            ASSERT_TRUE(model_ptr->getVelocityTwist(link->name, twist));
            ASSERT_TRUE(model_ptr->getJacobian(link->name, J));
            actual_twist = J*qdot;
            EXPECT_NEAR((twist-actual_twist).norm(), 0.0, 1e-6);
            
        }
        
        
    }
    
}


TEST_F( testModelInterface, checkIMU )
{
    
    XBot::ImuSensor::ConstPtr const_imu = fb_model_ptr->getImu().begin()->second;
    auto imu = std::const_pointer_cast<XBot::ImuSensor>(const_imu);
    
    
    Eigen::Quaterniond qor;
    qor.coeffs() << 1,2,3,4;
    qor.normalize();
    
    Eigen::Matrix3d R(qor);
    
    Eigen::Vector3d omega;
    omega.setRandom();
    
    Eigen::Vector3d acc;
    
    imu->setImuData(R, acc, omega, 0);
    
    Eigen::VectorXd q, qdot;
    q.setRandom(fb_model_ptr->getJointNum());
    qdot.setRandom(fb_model_ptr->getJointNum());

    fb_model_ptr->setJointPosition(q);
    fb_model_ptr->setJointVelocity(qdot);
    fb_model_ptr->update();
    
    Eigen::Affine3d T_fb;
    Eigen::Vector6d twist_fb;
    
    fb_model_ptr->getFloatingBasePose(T_fb);
    fb_model_ptr->getFloatingBaseTwist(twist_fb);
    
    
    ASSERT_TRUE( fb_model_ptr->setFloatingBaseState(imu) );
    fb_model_ptr->update();
    
    
    Eigen::VectorXd qnew, qdotnew;
    fb_model_ptr->getJointPosition(qnew);
    fb_model_ptr->getJointVelocity(qdotnew);
    
    Eigen::Matrix3d actual_R;
    Eigen::Affine3d T;
    ASSERT_TRUE( fb_model_ptr->getFloatingBasePose(T) );
    actual_R = T.linear();
    
    Eigen::Vector6d actual_omega;
    fb_model_ptr->getFloatingBaseTwist(actual_omega);
    
    std::cout << R << std::endl;
    std::cout << actual_R << std::endl;
    
    std::cout << omega.transpose() << std::endl;
    std::cout << actual_omega.transpose() << std::endl;
    
    
    
    EXPECT_NEAR( (R*actual_R.transpose() - Eigen::Matrix3d::Identity()).norm() , 0, 0.001);
    
    EXPECT_NEAR( (R*omega - actual_omega.tail<3>()).norm() , 0, 0.001);
    
    EXPECT_NEAR( (T.translation()-T_fb.translation()).norm(), 0.0, 1e-10 );
    
    EXPECT_NEAR( (twist_fb.head(3)-actual_omega.head(3)).norm(), 0.0, 1e-10 );
    
    EXPECT_NEAR( (q-qnew).head(3).norm(), 0, 1e-10 );
    EXPECT_NEAR( (qdot-qdotnew).head(3).norm(), 0, 1e-10 );
    
    
    
}

void printKDLTwist(const std::string& text, const KDL::Twist& twist)
{
    Eigen::Vector6d tmp;
    tmp[0] = twist.vel.x();
    tmp[1] = twist.vel.y();
    tmp[2] = twist.vel.z();
    tmp[3] = twist.rot.x();
    tmp[4] = twist.rot.y();
    tmp[5] = twist.rot.z();

    std::cout<<text<<"\n"<<tmp.transpose()<<std::endl;
}

Eigen::VectorXd setSin(const double t, const int size)
{
    Eigen::VectorXd v(size);
    for(unsigned int i = 0; i < size; ++i)
    {
        v[i] = 0.1*i*sin(t)+0.1;
    }
    return v;
}



TEST_F(testModelInterface, getRelativeAccelerationTwist)
{
    double dT = 0.00001;
    Eigen::VectorXd q, qdot, qddot;
    //qdot.setOnes(fb_model_ptr->getJointNum());
    qdot = Eigen::VectorXd::Ones(fb_model_ptr->getJointNum());
    q.setRandom(fb_model_ptr->getJointNum()); //q(0) = random

    //1 sec random points
    std::vector<Eigen::VectorXd> qs, qdots, qddots;
    qs.push_back(q);
    qdots.push_back(qdot);
    for(unsigned int i = 0; i < 1000; ++i)
    {
        //qddot.setRandom(fb_model_ptr->getJointNum());
        //qddot = Eigen::VectorXd::Ones(qdot.size());
        qddot = setSin(i*dT, qdot.size());
        qddots.push_back(qddot);

        q += qdot*dT + 0.5*qddot*dT*dT;
        qdot += qddot*dT;

        qs.push_back(q);
        qdots.push_back(qdot);
    }

//    std::cout<<"qddots size: "<<qddots.size()<<std::endl;
//    std::cout<<"qdots size: "<<qdots.size()<<std::endl;
//    std::cout<<"qs size: "<<qs.size()<<std::endl;

//    std::cout<<std::endl;

    std::string base_link = "arm1_7";
    std::string distal_link = "arm2_7";

    for(unsigned int i = 0; i < qddots.size(); ++i)
    //for(unsigned int i = 0; i < 10; ++i)
    {
        unsigned int j = i+1;

        fb_model_ptr->setJointPosition(qs[i]);
        fb_model_ptr->setJointVelocity(qdots[i]);
        fb_model_ptr->setJointAcceleration(qddots[i]);
        fb_model_ptr->update();

        KDL::Twist acc, vel_i;
        fb_model_ptr->getRelativeVelocityTwist(distal_link, base_link, vel_i);
        fb_model_ptr->getRelativeAccelerationTwist(distal_link, base_link, acc);

        fb_model_ptr->setJointPosition(qs[j]);
        fb_model_ptr->setJointVelocity(qdots[j]);
        fb_model_ptr->update();

        KDL::Twist vel_j;
        fb_model_ptr->getRelativeVelocityTwist(distal_link, base_link, vel_j);

        KDL::Twist acc_derived = (vel_j - vel_i)/dT;

//        printKDLTwist("acc: ", acc);
//        printKDLTwist("acc_derived: ", acc_derived);
//        std::cout<<std::endl;

        EXPECT_NEAR(acc.vel.x(), acc_derived.vel.x(), 1e-3);
        EXPECT_NEAR(acc.vel.y(), acc_derived.vel.y(), 1e-3);
        EXPECT_NEAR(acc.vel.z(), acc_derived.vel.z(), 1e-3);
        EXPECT_NEAR(acc.rot.x(), acc_derived.rot.x(), 1e-3);
        EXPECT_NEAR(acc.rot.y(), acc_derived.rot.y(), 1e-3);
        EXPECT_NEAR(acc.rot.z(), acc_derived.rot.z(), 1e-3);

    }


}

TEST_F(testModelInterface, computeRelativeJDotQdot)
{
    double dT = 0.00001;
    Eigen::VectorXd q, qdot, qddot;
    //qdot.setOnes(fb_model_ptr->getJointNum());
    qdot = Eigen::VectorXd::Ones(fb_model_ptr->getJointNum());
    q.setRandom(fb_model_ptr->getJointNum()); //q(0) = random

    //1 sec random points
    std::vector<Eigen::VectorXd> qs, qdots, qddots;
    qs.push_back(q);
    qdots.push_back(qdot);
    for(unsigned int i = 0; i < 1000; ++i)
    {
        //qddot.setRandom(fb_model_ptr->getJointNum());
        //qddot = Eigen::VectorXd::Ones(qdot.size());
        qddot = setSin(i*dT, qdot.size());
        qddots.push_back(qddot);

        q += qdot*dT + 0.5*qddot*dT*dT;
        qdot += qddot*dT;

        qs.push_back(q);
        qdots.push_back(qdot);
    }

//    std::cout<<"qddots size: "<<qddots.size()<<std::endl;
//    std::cout<<"qdots size: "<<qdots.size()<<std::endl;
//    std::cout<<"qs size: "<<qs.size()<<std::endl;

//    std::cout<<std::endl;

    std::string base_link = "arm1_7";
    std::string distal_link = "arm2_7";

    for(unsigned int i = 0; i < qddots.size(); ++i)
    {
        fb_model_ptr->setJointPosition(qs[i]);
        fb_model_ptr->setJointVelocity(qdots[i]);
        fb_model_ptr->setJointAcceleration(qddots[i]);
        fb_model_ptr->update();

        Eigen::Vector6d acc;
        fb_model_ptr->getRelativeAccelerationTwist(distal_link, base_link, acc);

        Eigen::Vector6d JDotQDot;
        fb_model_ptr->computeRelativeJdotQdot(distal_link, base_link, JDotQDot);

        Eigen::MatrixXd J;
        fb_model_ptr->getRelativeJacobian(distal_link, base_link, J);

        Eigen::Vector6d JDotQDot_ = acc - J*qddots[i];


        EXPECT_NEAR(JDotQDot_[0], JDotQDot[0], 1e-9);
        EXPECT_NEAR(JDotQDot_[1], JDotQDot[1], 1e-9);
        EXPECT_NEAR(JDotQDot_[2], JDotQDot[2], 1e-9);
        EXPECT_NEAR(JDotQDot_[3], JDotQDot[3], 1e-9);
        EXPECT_NEAR(JDotQDot_[4], JDotQDot[4], 1e-9);
        EXPECT_NEAR(JDotQDot_[5], JDotQDot[5], 1e-9);


    }


}


int main ( int argc, char **argv )
{
     testing::InitGoogleTest ( &argc, argv );
     return RUN_ALL_TESTS();
}
