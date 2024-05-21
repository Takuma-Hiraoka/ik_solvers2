#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <iostream>
#include <ros/package.h>

#include <sr_inverse_kinematics_solver/sr_inverse_kinematics_solver.h>
#include <ik_constraint2/ik_constraint2.h>

namespace sr_inverse_kinematics_solver_sample{
  void sample1(){
    // load robot
    std::string modelfile = ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body";
    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr robot = bodyLoader.load(modelfile);

    // reset manip pose
    std::vector<double> reset_manip_pose{
      0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// rleg
        0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045,// rarm
        0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// lleg
        0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045,// larm
        0.0, 0.0, 0.0};

    for(int j=0; j < robot->numJoints(); ++j){
      robot->joint(j)->q() = reset_manip_pose[j];
    }
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();

    // setup tasks
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
    {
      // task: rarm to target
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->link("RARM_WRIST_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.3,-0.2,0.8);
      constraint->B_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(-1.5,cnoid::Vector3(0,1,0)));
      constraints.push_back(constraint);
    }
    {
      // task: larm to target. rotation-axis nil
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->link("LARM_WRIST_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.3,0.2,0.8);
      constraint->B_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(-1.5,cnoid::Vector3(0,1,0)));
      for(size_t i=0;i<3;i++)constraint->weight()[3+i] = 0.0;
      constraints.push_back(constraint);
    }
    {
      // task: rleg to target
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->link("RLEG_ANKLE_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,-0.2,0.0);
      constraints.push_back(constraint);
    }
    {
      // task: lleg to target
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->link("LLEG_ANKLE_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,0.2,0.0);
      constraints.push_back(constraint);
    }
    {
      // task: COM to target
      std::shared_ptr<ik_constraint2::COMConstraint> constraint = std::make_shared<ik_constraint2::COMConstraint>();
      constraint->A_robot() = robot;
      constraint->B_localp() = cnoid::Vector3(0.0,0.0,0.7);
      constraints.push_back(constraint);
    }
    {
      // task: joint angle to target
      std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
      constraint->joint() = robot->link("CHEST");
      constraint->targetq() = 0.1;
      constraints.push_back(constraint);
    }

    for(size_t i=0;i<constraints.size();i++) constraints[i]->debugLevel() = 1;//debug

    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(robot->rootLink());
    for(int i=0;i<robot->numJoints();i++) variables.push_back(robot->joint(i));
    sr_inverse_kinematics_solver::IKParam param;
    param.maxIteration = 1;
    param.debugLevel = 1;

    // setup viewer
    choreonoid_viewer::Viewer viewer;
    viewer.objects(robot);
    viewer.drawObjects();
    getchar();

    // main loop
    bool solved = false;
    for(int i=0;i<200;i++){
      solved = sr_inverse_kinematics_solver::solveIKLoop(variables,
                                                         constraints,
                                                         param);

      if(i % 1 == 0){
        std::cerr << "loop: " << i << std::endl;
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints.size();j++){
          const std::vector<cnoid::SgNodePtr>& marker = constraints[j]->getDrawOnObjects();
          std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
        }
        viewer.drawOn(markers);
        viewer.drawObjects();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      if(solved) break;
    }

    std::cerr << "solved: " << solved << std::endl;

    for(size_t i=0;i<constraints.size();i++){
      constraints[i]->debugLevel() = 0;//not debug
      if(constraints[i]->isSatisfied()) std::cerr << "constraint " << i << ": converged"<< std::endl;
      else std::cerr << "constraint " << i << ": NOT converged"<< std::endl;
    }

    return;
  }
}
