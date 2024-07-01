#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <iostream>
#include <ros/package.h>

#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>
#include <ik_constraint2/ik_constraint2.h>
#include <ik_constraint2_esdf/ik_constraint2_esdf.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/integrator/esdf_integrator.h>

#include <cnoid/TimeMeasure>

namespace prioritized_inverse_kinematics_solver2_sample_esdf{
  void sample1_4limb(){
    cnoid::TimeMeasure timer; timer.begin();

    // load robot
    std::string modelfile = ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body";
    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr robot = bodyLoader.load(modelfile);
    std::string deskModelfile = ros::package::getPath("prioritized_inverse_kinematics_solver2_sample_esdf") + "/models/desk.wrl";
    cnoid::BodyPtr desk = bodyLoader.load(deskModelfile);

    // reset manip pose
    robot->rootLink()->p() = cnoid::Vector3(0,0,0.6);
    robot->rootLink()->v().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();
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

    desk->rootLink()->p() = cnoid::Vector3(0.75,0.0,0.5);
    desk->rootLink()->R() = cnoid::AngleAxisd(0.2, cnoid::Vector3::UnitX()).toRotationMatrix();
    desk->calcForwardKinematics();
    desk->calcCenterOfMass();

    // setup tasks
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    // joint limit
    for(int i=0;i<robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = robot->joint(i);
      constraints0.push_back(constraint);
    }

    std::cerr << "before sdf: " << timer.measure() << "[s]." << std::endl;
    {
      // task: env collision

      voxblox::TsdfMap::Config tsdf_config;
      tsdf_config.tsdf_voxel_size = 0.02;
      std::shared_ptr<voxblox::TsdfMap> tsdf_map = std::make_shared<voxblox::TsdfMap>(tsdf_config);
      voxblox::TsdfIntegratorBase::Config tsdf_integrator_config;
      tsdf_integrator_config.voxel_carving_enabled = true;
      tsdf_integrator_config.default_truncation_distance = 0.02;
      tsdf_integrator_config.max_weight = 30.0;
      tsdf_integrator_config.min_ray_length_m = 0.01;
      std::shared_ptr<voxblox::FastTsdfIntegrator> tsdfIntegrator = std::make_shared<voxblox::FastTsdfIntegrator>(tsdf_integrator_config, tsdf_map->getTsdfLayerPtr());
      //std::shared_ptr<voxblox::MergedTsdfIntegrator> tsdfIntegrator = std::make_shared<voxblox::MergedTsdfIntegrator>(tsdf_integrator_config, tsdf_map->getTsdfLayerPtr());

      std::cerr << "after tsdf init: " << timer.measure() << "[s]." << std::endl;

      voxblox::EsdfMap::Config esdf_config;
      esdf_config.esdf_voxel_size = tsdf_config.tsdf_voxel_size;
      std::shared_ptr<voxblox::EsdfMap> esdf_map = std::make_shared<voxblox::EsdfMap>(esdf_config);
      voxblox::EsdfIntegrator::Config esdf_integrator_config;
      esdf_integrator_config.min_distance_m = 0.02;
      esdf_integrator_config.max_distance_m = 0.5;
      esdf_integrator_config.default_distance_m = esdf_integrator_config.max_distance_m;
      esdf_integrator_config.clear_sphere_radius = 3.0;
      //esdf_integrator_config.full_euclidean_distance = true;
      std::shared_ptr<voxblox::EsdfIntegrator> esdfIntegrator = std::make_shared<voxblox::EsdfIntegrator>(esdf_integrator_config, tsdf_map->getTsdfLayerPtr(), esdf_map->getEsdfLayerPtr());
      esdfIntegrator->addNewRobotPosition(voxblox::Point(0.0, 0.0, 0.0)); // clearする.

      std::cerr << "after esdf init: " << timer.measure() << "[s]." << std::endl;

      double ray = 0.1;
      for(int i=0;i<desk->numLinks();i++){
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > vertices = ik_constraint2_esdf::getSurfaceVerticesAndNormals(desk->link(i), 0.02, M_PI/3); // link local
        for(int j=0;j<vertices.size();j++){
          cnoid::Vector3 p = desk->link(i)->T() * vertices[j].first; // world frame
          cnoid::Vector3 n = (desk->link(i)->R() * vertices[j].second).normalized(); // world frame

          cnoid::Vector3 origin = p + n * ray; // カメラ原点. world frame
          cnoid::Vector3 z = -n.normalized(); // カメラ姿勢. world frame
          cnoid::Vector3 x, y;
          if(cnoid::Vector3::UnitY().cross(z).norm() > 0){
            x = cnoid::Vector3::UnitY().cross(z).normalized();
            y = z.cross(x).normalized();
          }else{
            y = z.cross(cnoid::Vector3::UnitX()).normalized();
            x = y.cross(z).normalized();
          }
          cnoid::Matrix3 R; R.col(0) = x; R.col(1) = y; R.col(2) = z; // カメラ姿勢. world frame
          voxblox::Transformation trans(Eigen::Quaterniond(R).cast<float>(), origin.cast<float>());
          voxblox::Pointcloud pcl{Eigen::Vector3f(0.0,0.0,ray)};
          voxblox::Colors color{voxblox::Color(0.0,0.0,0.0)};
          tsdfIntegrator->integratePointCloud(trans, pcl, color);
        }
      }
      std::cerr << "after tsdf: " << timer.measure() << "[s]." << std::endl;

      esdfIntegrator->updateFromTsdfLayer(true);
      std::cerr << "after esdf: " << timer.measure() << "[s]." << std::endl;

      for(double z=0;z<2.0; z+=0.01){
        cnoid::Vector3 grad;
        double dist;
        bool success = esdf_map->getDistanceAndGradientAtPosition(cnoid::Vector3(1.0,0,z)+cnoid::Vector3(1e-3,1e-3,1e-3),true,&dist,&grad);
        std::cerr << z << " "<< success << " " << dist << " " << grad.transpose() << std::endl;
      }

      for(int i=0;i<robot->numLinks();i++){
        std::shared_ptr<ik_constraint2_esdf::EsdfCollisionConstraint> constraint = std::make_shared<ik_constraint2_esdf::EsdfCollisionConstraint>();
        constraint->A_link() = robot->link(i);
        constraint->field() = esdf_map;
        constraint->tolerance() = 0.03;
        constraint->maxDistance() = esdf_integrator_config.max_distance_m - 0.01;
        constraints0.push_back(constraint);
      }
    }


    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
    {
      // task: rleg to target
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->link("RLEG_ANKLE_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,-0.2,-0.0);
      constraints1.push_back(constraint);
    }
    {
      // task: lleg to target
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->link("LLEG_ANKLE_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,0.2,0.0);
      constraints1.push_back(constraint);
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2;
    {
      // task: rarm to target. never reach
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = robot->link("RARM_WRIST_R");
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation() = cnoid::Vector3(1.2,-0.2,0.8);
      constraint->B_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(-1.5,cnoid::Vector3(0,1,0)));
      constraints2.push_back(constraint);
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
      constraints2.push_back(constraint);
    }

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints3;
    {
      // task: joint angle to target
      std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
      constraint->joint() = robot->link("CHEST");
      constraint->targetq() = 0.1;
      constraints3.push_back(constraint);
    }


    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(robot->rootLink());
    for(size_t i=0;i<robot->numJoints();i++){
      variables.push_back(robot->joint(i));
    }
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1,constraints2,constraints3};
    for(size_t i=0;i<constraints.size();i++){
      for(size_t j=0;j<constraints[i].size();j++){
        constraints[i][j]->debugLevel() = 0;//debug
      }
    }

    // setup viewer
    choreonoid_viewer::Viewer viewer;
    viewer.objects(std::vector<cnoid::BodyPtr>{robot, desk});

    // main loop
    for(int i=0;i<200;i++){
      prioritized_inverse_kinematics_solver2::IKParam param;
      param.debugLevel = 0; // debug
      param.maxIteration = 1;
      param.we = 1e2;
      bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                                        constraints,
                                                                        tasks,
                                                                        param);

      if(i % 10 == 0){
        std::cerr << "loop: " << i << std::endl;
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints.size();j++){
          for(int k=0;k<constraints[j].size(); k++){
            const std::vector<cnoid::SgNodePtr>& marker = constraints[j][k]->getDrawOnObjects();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
        }
        viewer.drawOn(markers);
        viewer.drawObjects();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      if(solved) break;
    }

    for(size_t i=0;i<constraints.size();i++){
      for(size_t j=0;j<constraints[i].size();j++){
        constraints[i][j]->debugLevel() = 0;//not debug
        constraints[i][j]->updateBounds();
        if(constraints[i][j]->isSatisfied()) std::cerr << "constraint " << i << " " << j << ": satisfied"<< std::endl;
        else std::cerr << "constraint " << i << " " << j << ": NOT satidfied"<< std::endl;
      }
    }

    return;
  }
}
