#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/centroidal.hpp"

#include <iostream>

int main(int argc, char ** argv)
{
    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = std::string("../ur5_robot.urdf");
    const std::string urdf_filename_B = std::string("../BonnieURDF_latest.urdf");
    // Load the urdf model
    pinocchio::Model model;
    pinocchio::Model model_Bonnie;
    //pinocchio::buildModels::manipulator()

    pinocchio::JointModelFreeFlyer root_joint;

    pinocchio::urdf::buildModel(urdf_filename_B,root_joint,model_Bonnie);
  //  pinocchio::urdf::buildModel(urdf_filename_B,model_Bonnie);
    pinocchio::urdf::buildModel(urdf_filename,model);
    std::cout << "model name: " << model.name << std::endl;



   // pinocchio::buildModels::manipulator(model);
    pinocchio::Data data(model);
    pinocchio::Data data_B(model_Bonnie);

    Eigen::VectorXd q = pinocchio::neutral(model);
    Eigen::VectorXd v = Eigen::VectorXd::Ones(model.nv)*0.1;
    Eigen::VectorXd a = Eigen::VectorXd::Ones(model.nv)*1;

    pinocchio::crba(model,data,q);
    //std::cout << "tau = " << tau.transpose() << std::endl;

    /*
    pinocchio::nonLinearEffects(model,data,q,v);
    std::cout << "M = " << data.M << std::endl;
    std::cout << "C = " << data.C << std::endl;
    std::cout << "G = " << data.g << std::endl;
    std::cout << "nle = " << data.nle << std::endl;
     */

    pinocchio::computeCoriolisMatrix(model,data,q,v);
    std::cout<<"C="<<data.C<<std::endl;

    pinocchio::computeGeneralizedGravity(model,data,q);
    std::cout<<"G="<<data.g<<std::endl;
    std::cout << "M = " << data.M << std::endl;
    Eigen::Matrix<double,6,6> M_t_0;
    M_t_0=data.M.transpose();
    for (int i=0;i<6;i++)
        M_t_0(i,i)=0;

    auto tau_cal=(M_t_0+data.M)*a+data.C*v+data.g;
    std::cout<<tau_cal<<std::endl;

    const Eigen::VectorXd & tau = pinocchio::rnea(model,data,q,v,a);
    std::cout << "tau = " << tau.transpose() << std::endl;

    std::cout<<model.inertias[0]<<std::endl<<"=============Bonnie Test============="<<std::endl;
//============================== Bonnie Test ==============================
    Eigen::VectorXd q_B = pinocchio::neutral(model_Bonnie);
    Eigen::VectorXd v_B = Eigen::VectorXd::Ones(model_Bonnie.nv)*0.1;
    Eigen::VectorXd a_B = Eigen::VectorXd::Ones(model_Bonnie.nv)*1;

    pinocchio::computeGeneralizedGravity(model_Bonnie,data_B,q_B);
    std::cout<<"G="<<data_B.g<<std::endl;

    pinocchio::crba(model_Bonnie,data_B,q_B);
    std::cout << "M = " << data_B.M << std::endl;

    std::cout<<model_Bonnie.inertias[0]<<std::endl;

    std::cout<<pinocchio::computeTotalMass(model_Bonnie)<<std::endl;

   // pinocchio::computeCentroidalMomentum(model_Bonnie,data_B,q_B,v_B);
    pinocchio::computeAllTerms(model_Bonnie,data_B,q_B,v_B);

    std::cout<<"centroid inertia"<<data_B.Ig<<std::endl;
    std::cout<<model_Bonnie.nq<<" "<<model_Bonnie.nv<<std::endl;

    //for (int i=0;i<model_Bonnie.nv;i++)
    //    std::cout<<data_B.joints[i]<<std::endl;
}