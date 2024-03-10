#include "perception_planner.h"

#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/base/objectives/MinimizeContactObjective.h>
#include <ompl/base/objectives/VFMagnitudeOptimizationObjective.h>
#include <ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/ProjectionTypes.h>
#include <ompl/multilevel/datastructures/projections/ModelBased_RN_RM.h>
#include <ompl/multilevel/planners/qmp/QMPStar.h>
#include <ompl/multilevel/planners/qrrt/QRRTStar.h>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/planners/informedtrees/BITstar.h"
#include "ompl/geometric/planners/rrt/ContactTRRT.h"
#include "ompl/geometric/planners/rrt/InformedRRTstar.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"

using namespace std::chrono;

constexpr char LOGNAME[] = "perception_planner";

namespace tacbot {

PerceptionPlanner::PerceptionPlanner() : BasePlanner() {
  ROS_INFO_NAMED(LOGNAME, "contact_perception_->init()");
  contact_perception_ = std::make_shared<ContactPerception>();
}

void PerceptionPlanner::init() {
  BasePlanner::init();
  contact_perception_->init();
  setCollisionChecker("Bullet");
  setGoalState(2);
  setObstacleScene(3);
  sphericalCollisionPermission(true);
  tableCollisionPermission();
}

void PerceptionPlanner::setGoalState(std::size_t option) {
  switch (option) {
    default:
    case 1:
      joint_goal_pos_ =
          std::vector<double>{-1.0, 0.7, 0.7, -1.0, -0.7, 2.0, 0.0};
      break;
    case 2:
      joint_goal_pos_ = std::vector<double>{-2.0304, -1.44185, 1.4716, -1.72948,
                                            0.66649, 0.724355, 1.38011};
      break;
  }
}

void PerceptionPlanner::setObstacleScene(std::size_t option) {
  obstacles_.clear();
  ROS_INFO_NAMED(LOGNAME, "Obstacle scene option selected: %ld", option);
  switch (option) {
    case 0: {
      ROS_INFO_NAMED(LOGNAME, "No additional obstacles added to the scene.");
      break;
    }
    case 1: {
      tacbot::ObstacleGroup sphere_1;
      sphere_1.name = "sphere_1";
      sphere_1.radius = 0.1;
      sphere_1.center = Eigen::Vector3d{0.5, 0.0, 0.6};
      obstacles_.emplace_back(sphere_1);

      tacbot::ObstacleGroup sphere_2;
      sphere_2.name = "sphere_2";
      sphere_2.radius = 0.1;
      sphere_2.center = Eigen::Vector3d{0.45, -0.4, 0.6};
      obstacles_.emplace_back(sphere_2);
      break;
    }
    case 2: {
      tacbot::ObstacleGroup sphere_1;
      sphere_1.name = "sphere_1";
      sphere_1.radius = 0.1;
      sphere_1.cost = 10.0;
      sphere_1.center = Eigen::Vector3d{0.45, -0.2, 0.6};
      obstacles_.emplace_back(sphere_1);

      tacbot::ObstacleGroup sphere_2;
      sphere_2.name = "sphere_2";
      sphere_2.radius = 0.1;
      sphere_2.cost = 1.0;
      sphere_2.center = Eigen::Vector3d{0.45, -0.4, 0.6};
      obstacles_.emplace_back(sphere_2);

      tacbot::ObstacleGroup sphere_3;
      sphere_3.name = "sphere_3";
      sphere_3.radius = 0.1;
      sphere_3.cost = 1.0;
      sphere_3.center = Eigen::Vector3d{0.45, -0.3, 0.8};
      obstacles_.emplace_back(sphere_3);
      break;
    }
    case 3: {
      tacbot::ObstacleGroup sphere_1;
      sphere_1.name = "sphere_1";
      sphere_1.radius = 0.12;
      sphere_1.cost = 10.0;
      sphere_1.center = Eigen::Vector3d{0.45, 0.1, 0.35};
      obstacles_.emplace_back(sphere_1);
      break;
    }
    default:
      ROS_ERROR_NAMED(LOGNAME,
                      "Selected obstacle scene option is out of bounds: %ld",
                      option);
      break;
  }

  // std::cout << "num obstacles: " << obstacles_.size() << std::endl;

  for (auto& obstacle : obstacles_) {
    addPointObstacles(obstacle);
    contact_perception_->addSphere(obstacle);
  }
}

void PerceptionPlanner::addPointObstacles(tacbot::ObstacleGroup& obstacle) {
  const double inc = 0.4;
  for (double theta = 0; theta < 2 * M_PI; theta += inc) {
    for (double phi = 0; phi < 2 * M_PI; phi += inc) {
      double x = obstacle.radius * cos(phi) * sin(theta) + obstacle.center[0];
      double y = obstacle.radius * sin(phi) * sin(theta) + obstacle.center[1];
      double z = obstacle.radius * cos(theta) + obstacle.center[2];
      tacbot::PointObstacle pt_obs;
      pt_obs.pos = Eigen::Vector3d{x, y, z};
      obstacle.point_obstacles.emplace_back(pt_obs);
    }
  }
}

void PerceptionPlanner::changePlanner() {
  ROS_INFO_NAMED(LOGNAME, "changePlanner()");
  ompl::geometric::SimpleSetupPtr simple_setup = context_->getOMPLSimpleSetup();
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();
  ompl::base::PlannerPtr planner;

  ompl::geometric::PathSimplifierPtr simplifier =
      simple_setup->getPathSimplifier();

  std::function<Eigen::VectorXd(const ompl::base::State*,
                                const ompl::base::State*)>
      vFieldFuncDuo;

  if (planner_name_ == "BITstar") {
    std::function<double(const ompl::base::State*)> optFunc;

    optFunc = std::bind(&PerceptionPlanner::overlapMagnitude, this,
                        std::placeholders::_1);
    optimization_objective_ =
        std::make_shared<ompl::base::MinimizeContactObjective>(si, optFunc);

    // simple_setup->setOptimizationObjective(optimization_objective_);

    simple_setup->setOptimizationObjective(
        std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));

    planner = std::make_shared<ompl::geometric::BITstar>(si);

  } else if (planner_name_ == "RRTstar") {
    std::function<double(const ompl::base::State*)> optFunc;

    optFunc = std::bind(&PerceptionPlanner::overlapMagnitude, this,
                        std::placeholders::_1);
    optimization_objective_ =
        std::make_shared<ompl::base::MinimizeContactObjective>(si, optFunc);
    simple_setup->setOptimizationObjective(optimization_objective_);

    planner = std::make_shared<ompl::geometric::RRTstar>(si);

  } else if (planner_name_ == "QRRTStar") {
    ROS_INFO_NAMED(LOGNAME, "createPandaBundleContext()");
    createPandaBundleContext();

    ROS_INFO_NAMED(LOGNAME, "setting siVec");
    std::vector<ompl::base::SpaceInformationPtr> siVec;
    ompl::base::SpaceInformationPtr BaseSi =
        pandaBundleContext_->getPlanningContext()
            ->getOMPLSimpleSetup()
            ->getSpaceInformation();
    ompl::base::SpaceInformationPtr BundleSi = si;
    siVec.emplace_back(BaseSi);
    siVec.emplace_back(BundleSi);

    ROS_INFO_NAMED(LOGNAME, "setting projVec");
    std::vector<ompl::multilevel::ProjectionPtr> projVec;
    ompl::base::StateSpacePtr BaseSpace =
        pandaBundleContext_->getPlanningContext()
            ->getOMPLSimpleSetup()
            ->getStateSpace();
    ompl::base::StateSpacePtr BundleSpace = simple_setup->getStateSpace();
    ompl::multilevel::ProjectionPtr projection =
        std::make_shared<ompl::multilevel::Projection_ModelBased_RN_RM>(
            BundleSpace, BaseSpace);
    ROS_INFO_NAMED(LOGNAME, "FiberedProjection");
    auto componentFiber =
        std::dynamic_pointer_cast<ompl::multilevel::FiberedProjection>(
            projection);
    ROS_INFO_NAMED(LOGNAME, "makeFiberSpace()");
    componentFiber->makeFiberSpace();
    projVec.emplace_back(projection);

    std::function<double(const ompl::base::State*)> optFunc;
    optFunc = std::bind(&PerceptionPlanner::overlapMagnitude, this,
                        std::placeholders::_1);
    optimization_objective_ =
        std::make_shared<ompl::base::MinimizeContactObjective>(si, optFunc);
    simple_setup->setOptimizationObjective(optimization_objective_);

    ROS_INFO_NAMED(LOGNAME, "ompl::multilevel::QRRTStar(siVec, projVec)");
    planner = std::make_shared<ompl::multilevel::QRRTStar>(siVec, projVec);

  } else if (planner_name_ == "CAT-TRRT") {
    ROS_INFO_NAMED(LOGNAME, "Using planner: CAT-TRRT.");
    std::function<Eigen::VectorXd(const ompl::base::State*,
                                  const ompl::base::State*)>
        vFieldFuncDuo;

    // vFieldFuncDuo = std::bind(&PerceptionPlanner::obstacleFieldDuo, this,
    //                           std::placeholders::_1, std::placeholders::_2);
    vFieldFuncDuo = std::bind(&PerceptionPlanner::obstacleFieldCartesian, this,
                              std::placeholders::_1, std::placeholders::_2);

    std::function<Eigen::VectorXd(const ompl::base::State*)> vFieldFunc;
    vFieldFunc = std::bind(&PerceptionPlanner::obstacleField, this,
                           std::placeholders::_1);

    optimization_objective_ =
        std::make_shared<ompl::base::VFUpstreamCriterionOptimizationObjective>(
            si, vFieldFunc);

    // optimization_objective_ =
    //     std::make_shared<ompl::base::VFMagnitudeOptimizationObjective>(
    //         si, vFieldFunc);

    simple_setup->setOptimizationObjective(optimization_objective_);
    planner = std::make_shared<ompl::geometric::ContactTRRT>(si, vFieldFuncDuo);

  } else if (planner_name_ == "RRTConnect") {
    ROS_INFO_NAMED(LOGNAME, "Using planner: RRTConnect.");
    planner = std::make_shared<ompl::geometric::RRTConnect>(si);
  } else {
    ROS_ERROR_NAMED(LOGNAME, "The following planner is not supported: %s",
                    planner_name_.c_str());
    throw std::invalid_argument(planner_name_);
  }

  // optimization_objective_->setCostToGoHeuristic(
  //     &ompl::base::goalRegionCostToGo);
  simple_setup->setPlanner(planner);
}

void PerceptionPlanner::extractPtsFromModel(
    const moveit::core::RobotStatePtr& robot_state,
    const moveit::core::LinkModel* link_model,
    std::vector<Eigen::Vector3d>& link_pts, std::size_t& num_pts) {
  std::vector<shapes::ShapeConstPtr> shapes = link_model->getShapes();
  std::size_t num_shapes = shapes.size();
  for (std::size_t j = 0; j < num_shapes; ++j) {
    Eigen::Isometry3d transform =
        robot_state->getCollisionBodyTransform(link_model, j);

    if (shapes[j]->type != shapes::MESH) {
      // should somehow extract points from this as well
      // ROS_ERROR_NAMED(LOGNAME, "Not a mesh shape");
    } else {
      shapes::Mesh* mesh = static_cast<shapes::Mesh*>(shapes[j]->clone());
      mesh->mergeVertices(0.05);
      // std::vector<Eigen::Vector3d> points(mesh->vertex_count);

      // std::vector<Eigen::Vector3d> points(mesh->vertex_count);

      for (unsigned int k = 0; k < mesh->vertex_count; ++k) {
        Eigen::Vector3d mesh_pt(mesh->vertices[3 * k],
                                mesh->vertices[3 * k + 1],
                                mesh->vertices[3 * k + 2]);
        // points[k] = transform * mesh_pt;
        link_pts.emplace_back(transform * mesh_pt);
        num_pts++;
      }

      // for (std::size_t p = 0; p < mesh->triangle_count; ++p) {
      //   std::size_t i3 = p * 3;
      //   Eigen::Vector3d s1(mesh->vertices[mesh->triangles[i3] * 3],
      //                      mesh->vertices[mesh->triangles[i3] * 3 + 1],
      //                      mesh->vertices[mesh->triangles[i3] * 3 + 2]);

      //   Eigen::Vector3d s2(mesh->vertices[mesh->triangles[i3 + 1] * 3],
      //                      mesh->vertices[mesh->triangles[i3 + 1] * 3 + 1],
      //                      mesh->vertices[mesh->triangles[i3 + 1] * 3 +
      //                      2]);

      //   Eigen::Vector3d s3(mesh->vertices[mesh->triangles[i3 + 2] * 3],
      //                      mesh->vertices[mesh->triangles[i3 + 2] * 3 + 1],
      //                      mesh->vertices[mesh->triangles[i3 + 2] * 3 +
      //                      2]);
      //   points.emplace_back(transform * ((s1 + s2) / 2));
      //   points.emplace_back(transform * ((s2 + s3) / 2));
      //   points.emplace_back(transform * ((s3 + s1) / 2));
      //   num_pts = num_pts + 3;
      // }
    }
  }
}

std::size_t PerceptionPlanner::getPtsOnRobotSurface(
    const moveit::core::RobotStatePtr& robot_state,
    std::vector<std::vector<Eigen::Vector3d>>& rob_pts) {
  const std::vector<std::string> link_names =
      joint_model_group_->getLinkModelNames();

  rob_pts.clear();
  std::size_t num_links = link_names.size();
  std::size_t num_pts = 0;
  // std::cout << "num_links " << num_links << std::endl;

  for (std::size_t i = 0; i < num_links; i++) {
    // std::cout << "i: " << i << ", " << link_names[i] << std::endl;

    const moveit::core::LinkModel* link_model =
        robot_state->getLinkModel(link_names[i]);

    const moveit::core::LinkTransformMap fixed_links =
        link_model->getAssociatedFixedTransforms();

    // std::cout << "link_tf_map.size(): " << link_tf_map.size() << std::endl;

    std::vector<shapes::ShapeConstPtr> shapes = link_model->getShapes();
    std::size_t num_shapes = shapes.size();

    std::vector<Eigen::Vector3d> link_pts;

    // the last link does not have a mesh for some reason
    if (i >= dof_ - 1 || (num_shapes == 0 && fixed_links.size() > 0)) {
      // std::cout << "i: " << i << ", " << link_names[i] << std::endl;
      for (const std::pair<const moveit::core::LinkModel* const,
                           Eigen::Isometry3d>& fixed_link : fixed_links) {
        extractPtsFromModel(robot_state, fixed_link.first, link_pts, num_pts);
      }

      if (i == num_links - 1) {
        // std::cout << "i: " << i << ", " << link_names[i] << std::endl;
        rob_pts.emplace_back(link_pts);
      }
      continue;
    }

    extractPtsFromModel(robot_state, link_model, link_pts, num_pts);
    rob_pts.emplace_back(link_pts);
  }
  // std::cout << "rob_pts.size() " << rob_pts.size() << std::endl;
  return num_pts;
}

Eigen::VectorXd PerceptionPlanner::getRobtPtsVecDiffAvg(
    const std::vector<std::vector<Eigen::Vector3d>>& near_state_rob_pts,
    const std::vector<std::vector<Eigen::Vector3d>>& rand_state_rob_pts,
    const std::vector<Eigen::Vector3d>& link_to_obs_vec) {
  std::size_t num_links = near_state_rob_pts.size();
  Eigen::VectorXd mean_per_link = Eigen::VectorXd::Zero(num_links);

  std::size_t pt_num = 0;

  for (std::size_t i = 0; i < num_links; i++) {
    std::vector<Eigen::Vector3d> near_pts_on_link = near_state_rob_pts[i];
    std::vector<Eigen::Vector3d> rand_pts_on_link = rand_state_rob_pts[i];

    std::size_t num_pts_on_link = near_pts_on_link.size();
    Eigen::MatrixXd diff_per_link = Eigen::MatrixXd::Zero(num_pts_on_link, 3);

    for (std::size_t j = 0; j < num_pts_on_link; j++) {
      Eigen::Vector3d near_pt_on_rob = near_pts_on_link[j];
      Eigen::Vector3d rand_pt_on_rob = rand_pts_on_link[j];
      Eigen::Vector3d nearrand_diff = rand_pt_on_rob - near_pt_on_rob;
      nearrand_diff.normalize();
      nearrand_diff *= 0.05;

      diff_per_link(j, 0) = nearrand_diff[0];
      diff_per_link(j, 1) = nearrand_diff[1];
      diff_per_link(j, 2) = nearrand_diff[2];

      // std::cout << "pt_num: " << pt_num << std::endl;
      // std::cout << "sample_state_count_: " << sample_state_count_ <<
      // std::endl;

      vis_data_->saveNearRandVec(near_pt_on_rob, nearrand_diff, pt_num,
                                 sample_state_count_);

      pt_num++;
    }

    double av_x = diff_per_link.col(0).mean();
    double av_y = diff_per_link.col(1).mean();
    double av_z = diff_per_link.col(2).mean();

    Eigen::Vector3d nearrand_av(av_x, av_y, av_z);
    Eigen::Vector3d link_to_obs = link_to_obs_vec[i];

    // std::cout << "nearrand_av: " << nearrand_av.transpose() << std::endl;
    // std::cout << "link_to_obs: " << link_to_obs.transpose() << std::endl;
    double dot = link_to_obs.dot(nearrand_av);
    // std::cout << "dot: " << dot << std::endl;

    mean_per_link[i] = dot;
  }
  vis_data_->saveNearRandDot(mean_per_link);
  return mean_per_link;
}

Eigen::VectorXd PerceptionPlanner::obstacleFieldCartesian(
    const ompl::base::State* near_state, const ompl::base::State* rand_state) {
  const ompl::base::RealVectorStateSpace::StateType& vec_state1 =
      *near_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles1 = utilities::toStlVec(vec_state1, dof_);

  moveit::core::RobotStatePtr robot_state1 =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state1->setJointGroupPositions(joint_model_group_, joint_angles1);
  std::vector<std::vector<Eigen::Vector3d>> near_rob_pts;
  std::size_t num_pts = getPtsOnRobotSurface(robot_state1, near_rob_pts);

  // vis_data_->setTotalNumRepulsePts(num_pts);
  // std::cout << "num pts from near state: " << num_pts << std::endl;
  std::vector<Eigen::Vector3d> link_to_obs_vec = getLinkToObsVec(near_rob_pts);
  // std::cout << "link_to_obs_vec.size(): " << link_to_obs_vec.size()
  //           << std::endl;

  const ompl::base::RealVectorStateSpace::StateType& vec_state2 =
      *rand_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles2 = utilities::toStlVec(vec_state2, dof_);

  moveit::core::RobotStatePtr robot_state2 =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state2->setJointGroupPositions(joint_model_group_, joint_angles2);
  std::vector<std::vector<Eigen::Vector3d>> rand_rob_pts;
  num_pts = getPtsOnRobotSurface(robot_state2, rand_rob_pts);
  // std::cout << "num pts from rand state: " << num_pts << std::endl;

  Eigen::VectorXd vfield =
      getRobtPtsVecDiffAvg(near_rob_pts, rand_rob_pts, link_to_obs_vec);

  // vis_data_->saveRepulseAngles(joint_angles1, joint_angles2);
  sample_state_count_++;
  return vfield;
}

Eigen::Vector3d PerceptionPlanner::getAttractPt(std::size_t link_num,
                                                std::size_t pt_num) {
  std::vector<Eigen::Vector3d> pts_on_link = goal_rob_pts_[link_num];
  Eigen::Vector3d pt_on_rob = pts_on_link[pt_num];
  return pt_on_rob;
}

Eigen::Vector3d PerceptionPlanner::scaleToDist(Eigen::Vector3d vec) {
  double y_max = 3.0;
  double D = 50.0;
  // double dist_max = 1.0;
  // std::cout << "vec.norm() " << vec.norm() << std::endl;
  Eigen::Vector3d vec_out = Eigen::VectorXd::Zero(3);

  // double norm = vec.norm();
  // double norm_max = 0.1;
  // if (norm < norm_max) {
  //   vec_out[0] = 1;
  //   vec_out[1] = 1;
  //   vec_out[2] = 1;
  // } else {
  //   vec_out[0] = 0;
  //   vec_out[1] = 0;
  //   vec_out[2] = 0;
  // }

  double prox_radius = 0.02;
  if (planner_name_ == "ContactTRRTDuo" || planner_name_ == "VFRRT") {
    prox_radius = 0.2;
  }

  if (vec.squaredNorm() > prox_radius) {
    return vec_out;
  }

  if (planner_name_ != "ContactTRRTDuo") {
    vec.normalize();
    return vec;
  }

  vec_out = (vec * y_max) / (D * vec.squaredNorm() + 1.0);

  // std::cout << "vec.squaredNorm() " << vec.squaredNorm() << std::endl;
  // std::cout << "before scale " << vec.transpose() << std::endl;
  // std::cout << "vec_out " << vec_out.transpose() << std::endl;
  return vec_out;
}

std::vector<Eigen::Vector3d> PerceptionPlanner::getObstacles(
    const Eigen::Vector3d& pt_on_rob) {
  std::vector<Eigen::Vector3d> obstacles;
  if (use_sim_obstacles_) {
    vis_data_->saveObstaclePos(sim_obstacle_pos_, sample_state_count_);
    return sim_obstacle_pos_;
  }

  bool status = contact_perception_->extractNearPts(pt_on_rob, obstacles);
  if (status) {
    vis_data_->saveObstaclePos(obstacles, sample_state_count_);
    return obstacles;
  }

  // Store an empty obstacle vector when no obstacles have been found in
  // proximity. This ensures that the size of the stored obstacles in an array
  // are equal to the number of states that we considered.
  vis_data_->saveObstaclePos(std::vector<Eigen::Vector3d>{},
                             sample_state_count_);

  // This yields a zero repulsion vector and will mean no repulsion will be
  // applied by the vectors filed.
  obstacles.emplace_back(pt_on_rob);

  return obstacles;
}

std::vector<Eigen::Vector3d> PerceptionPlanner::getLinkToObsVec(
    const std::vector<std::vector<Eigen::Vector3d>>& rob_pts) {
  std::size_t num_links = rob_pts.size();

  std::vector<Eigen::Vector3d> link_to_obs_vec(num_links,
                                               Eigen::Vector3d::Zero(3));

  std::size_t pt_num = 0;

  for (std::size_t i = 0; i < num_links; i++) {
    std::vector<Eigen::Vector3d> pts_on_link = rob_pts[i];
    std::size_t num_pts_on_link = pts_on_link.size();
    // std::cout << "link_num: " << i << std::endl;
    // std::cout << "num_pts_on_link: " << num_pts_on_link << std::endl;

    Eigen::MatrixXd pts_link_vec = Eigen::MatrixXd::Zero(num_pts_on_link, 3);

    // std::cout << "getObstacles: " << std::endl;
    std::vector<Eigen::Vector3d> obstacle_pos = getObstacles(pts_on_link[0]);

    for (std::size_t j = 0; j < num_pts_on_link; j++) {
      // std::cout << "pt j: " << j << std::endl;

      Eigen::Vector3d pt_on_rob = pts_on_link[j];
      // std::cout << "pt_on_rob: " << pt_on_rob.transpose() << std::endl;

      // std::vector<Eigen::Vector3d> obstacle_pos = getObstacles(pt_on_rob);

      std::size_t num_obstacles = obstacle_pos.size();
      // std::cout << "num_obstacles: " << num_obstacles << std::endl;

      Eigen::MatrixXd pt_to_obs = Eigen::MatrixXd::Zero(num_obstacles, 3);
      for (std::size_t k = 0; k < num_obstacles; k++) {
        Eigen::Vector3d vec = pt_on_rob - obstacle_pos[k];
        // std::cout << "vec: " << vec.transpose() << std::endl;
        // std::cout << "obstacle_pos: " << obstacle_pos[k].transpose()
        //           << std::endl;
        // std::cout << "pt_on_rob: " << pt_on_rob.transpose() << std::endl;

        vec = scaleToDist(vec);
        // std::cout << "vec: " << vec.transpose() << std::endl;

        if (planner_name_ == "ContactTRRTDuo") {
          Eigen::Vector3d att_pt = getAttractPt(i, j);

          Eigen::Vector3d att_vec = att_pt - pt_on_rob;

          if (vec.norm() == 0.0) {
            att_vec = Eigen::VectorXd::Zero(3);
          } else {
            att_vec.normalize();
          }

          double dot = att_vec.dot(vec);
          if (dot < -0.5) {
            att_vec = Eigen::VectorXd::Zero(3);
          }

          vec = vec + 0.5 * att_vec;
          // std::cout << "vec: " << vec.transpose() << std::endl;
        }

        pt_to_obs(k, 0) = vec[0];
        pt_to_obs(k, 1) = vec[1];
        pt_to_obs(k, 2) = vec[2];
      }

      double av_x = pt_to_obs.col(0).mean();
      double av_y = pt_to_obs.col(1).mean();
      double av_z = pt_to_obs.col(2).mean();

      Eigen::Vector3d pt_to_obs_av(av_x, av_y, av_z);
      // std::cout << "pt_to_obs_av: " << pt_to_obs_av.transpose() << std::endl;
      // std::cout << "pt_num: " << pt_num << std::endl;
      // std::cout << "sample_state_count_: " << sample_state_count_ <<
      // std::endl; std::cout << "pt_on_rob: " << pt_on_rob.transpose() <<
      // std::endl;

      pts_link_vec(j, 0) = pt_to_obs_av[0];
      pts_link_vec(j, 1) = pt_to_obs_av[1];
      pts_link_vec(j, 2) = pt_to_obs_av[2];

      // std::cout << "saveOriginVec: " << std::endl;

      vis_data_->saveOriginVec(pt_on_rob, pt_to_obs_av, pt_num,
                               sample_state_count_);
      pt_num++;
    }
    // std::cout << "pts_link_vec.rows(): " << pts_link_vec.rows() << std::endl;
    double av_x = pts_link_vec.col(0).mean();
    double av_y = pts_link_vec.col(1).mean();
    double av_z = pts_link_vec.col(2).mean();
    Eigen::Vector3d link_to_obs_avg(av_x, av_y, av_z);
    // std::cout << "link_to_obs_avg: " << link_to_obs_avg.transpose()
    //           << std::endl;

    link_to_obs_vec[i] = link_to_obs_avg;
  }
  vis_data_->saveAvgRepulseVec(link_to_obs_vec);

  return link_to_obs_vec;
}

Eigen::VectorXd PerceptionPlanner::obstacleFieldDuo(
    const ompl::base::State* near_state, const ompl::base::State* rand_state) {
  // one state not used
  return obstacleField(rand_state);
}

Eigen::VectorXd PerceptionPlanner::obstacleField(
    const ompl::base::State* rand_state) {
  sphericalCollisionPermission(false);
  const ompl::base::RealVectorStateSpace::StateType& vec_state =
      *rand_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles = utilities::toStlVec(vec_state, dof_);
  moveit::core::RobotState robot_state(*robot_state_);
  robot_state.setJointGroupPositions(joint_model_group_, joint_angles);
  Eigen::VectorXd vfield = getPerLinkContactDepth(robot_state);
  sphericalCollisionPermission(true);
  return vfield;
}

double PerceptionPlanner::overlapMagnitude(const ompl::base::State* state) {
  sphericalCollisionPermission(false);

  const ompl::base::RealVectorStateSpace::StateType& vec_state =
      *state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles = utilities::toStlVec(vec_state, dof_);
  moveit::core::RobotState robot_state(*robot_state_);
  robot_state.setJointGroupPositions(joint_model_group_, joint_angles);
  double cost = getContactDepth(robot_state);

  sphericalCollisionPermission(true);
  return cost;
}

void PerceptionPlanner::sphericalCollisionPermission(bool is_allowed) {
  // collision_detection::CollisionEnvConstPtr col_env =
  //     planning_scene_monitor::LockedPlanningSceneRW(psm_)->getCollisionEnv();
  collision_detection::AllowedCollisionMatrix& acm =
      planning_scene_monitor::LockedPlanningSceneRW(psm_)
          ->getAllowedCollisionMatrixNonConst();

  std::vector<std::string> sphere_names;
  for (auto obstacle : obstacles_) {
    sphere_names.emplace_back(obstacle.name);
  }

  for (auto name : sphere_names) {
    acm.setEntry(name, is_allowed);
  }
}

void PerceptionPlanner::tableCollisionPermission() {
  collision_detection::AllowedCollisionMatrix& acm =
      planning_scene_monitor::LockedPlanningSceneRW(psm_)
          ->getAllowedCollisionMatrixNonConst();
  std::string name = "table";
  std::vector<std::string> other_names{"panda_link0"};
  acm.setEntry(name, other_names, true);
}

void PerceptionPlanner::setCollisionChecker(
    std::string collision_checker_name) {
  planning_scene_monitor::LockedPlanningSceneRW(psm_)->addCollisionDetector(
      collision_detection::CollisionDetectorAllocatorBullet::create());
  planning_scene_monitor::LockedPlanningSceneRW(psm_)
      ->setActiveCollisionDetector(collision_checker_name);

  const std::string active_col_det =
      planning_scene_monitor::LockedPlanningSceneRO(psm_)
          ->getActiveCollisionDetectorName();
  ROS_INFO_NAMED(LOGNAME, "ActiveCollisionDetectorName: %s",
                 active_col_det.c_str());

  // collision_detection::CollisionEnvConstPtr col_env =
  //     planning_scene_monitor::LockedPlanningSceneRW(psm_)->getCollisionEnv();
  // std::vector<moveit_msgs::LinkPadding> padding;
  // col_env->getPadding(padding);
  // for (auto pad : padding) {
  //   std::cout << pad.link_name << ", " << pad.padding << std::endl;
  // }
}

Eigen::VectorXd PerceptionPlanner::getPerLinkContactDepth(
    moveit::core::RobotState robot_state) {
  collision_detection::CollisionRequest collision_request;
  collision_request.distance = false;
  collision_request.cost = false;
  collision_request.contacts = true;
  collision_request.max_contacts = 20;
  collision_request.max_contacts_per_pair = 1;
  collision_request.max_cost_sources = 20;
  collision_request.verbose = false;

  collision_detection::CollisionResult collision_result;
  planning_scene_monitor::LockedPlanningSceneRO(psm_)->checkCollisionUnpadded(
      collision_request, collision_result, robot_state);

  bool collision = collision_result.collision;
  // ROS_INFO_NAMED(LOGNAME, "collision: %d", collision);

  if (!collision) {
    return Eigen::VectorXd::Zero(dof_);
  }

  collision_detection::CollisionResult::ContactMap contact_map =
      collision_result.contacts;

  Eigen::VectorXd field_out = Eigen::VectorXd::Zero(dof_);

  for (auto contact : contact_map) {
    std::size_t num_subcontacts = contact.second.size();
    // ROS_INFO_NAMED(LOGNAME, "Number of subcontacts: %ld", num_subcontacts);
    for (std::size_t subc_idx = 0; subc_idx < num_subcontacts; subc_idx++) {
      collision_detection::Contact subcontact = contact.second[subc_idx];
      // ROS_INFO_NAMED(LOGNAME, "Body 1: %s", subcontact.body_name_1.c_str());
      // ROS_INFO_NAMED(LOGNAME, "Body 2: %s", subcontact.body_name_2.c_str());
      // ROS_INFO_NAMED(LOGNAME, "Depth: %f", subcontact.depth);

      std::size_t idx = 0;
      if (utilities::linkNameToIdx(subcontact.body_name_1, idx)) {
        field_out[idx] += std::abs(subcontact.depth);
      } else if (utilities::linkNameToIdx(subcontact.body_name_2, idx)) {
        field_out[idx] += std::abs(subcontact.depth);
      }
    }
  }

  return field_out;
}

double PerceptionPlanner::getContactDepth(
    moveit::core::RobotState robot_state) {
  // ROS_INFO_NAMED(LOGNAME, "getContactDepth");

  collision_detection::CollisionRequest collision_request;
  collision_request.distance = false;
  collision_request.cost = false;
  collision_request.contacts = true;
  collision_request.max_contacts = 20;
  collision_request.max_contacts_per_pair = 1;
  collision_request.max_cost_sources = 20;
  collision_request.verbose = false;

  collision_detection::CollisionResult collision_result;
  planning_scene_monitor::LockedPlanningSceneRO(psm_)->checkCollisionUnpadded(
      collision_request, collision_result, robot_state);

  bool collision = collision_result.collision;
  // ROS_INFO_NAMED(LOGNAME, "collision: %d", collision);

  if (!collision) {
    return 0;
  }

  collision_detection::CollisionResult::ContactMap contact_map =
      collision_result.contacts;

  double total_depth = 0;

  for (auto contact : contact_map) {
    std::size_t num_subcontacts = contact.second.size();
    // ROS_INFO_NAMED(LOGNAME, "Number of subcontacts: %ld",
    // num_subcontacts);
    for (std::size_t subc_idx = 0; subc_idx < num_subcontacts; subc_idx++) {
      collision_detection::Contact subcontact = contact.second[subc_idx];
      // ROS_INFO_NAMED(LOGNAME, "Body 1: %s",
      // subcontact.body_name_1.c_str());
      // ROS_INFO_NAMED(LOGNAME, "Body 2: %s",
      // subcontact.body_name_2.c_str());
      // ROS_INFO_NAMED(LOGNAME, "Depth: %f", subcontact.depth);

      // std::size_t idx = 0;
      // if (utilities::linkNameToIdx(subcontact.body_name_1, idx) ||
      //     utilities::linkNameToIdx(subcontact.body_name_2, idx)) {
      //   if (idx == 6) {
      //     subcontact.depth += 1;
      //   }
      // }

      tacbot::ObstacleGroup obstacle;
      if (findObstacleByName(subcontact.body_name_1, obstacle) ||
          findObstacleByName(subcontact.body_name_2, obstacle)) {
        total_depth = total_depth + std::abs(subcontact.depth) * obstacle.cost;
      } else {
        total_depth = total_depth + std::abs(subcontact.depth);
      }
    }
  }

  // ROS_INFO_NAMED(LOGNAME, "Total Contact Depth: %f", total_depth);
  return total_depth;
}

bool PerceptionPlanner::findObstacleByName(const std::string& name,
                                           tacbot::ObstacleGroup& obstacle) {
  auto it = std::find_if(std::begin(obstacles_), std::end(obstacles_),
                         [=](const tacbot::ObstacleGroup& obs) -> bool {
                           return name == obs.name;
                         });
  if (it == std::end(obstacles_)) {
    return false;
  } else {
    obstacle = *it;
    return true;
  }
}

void PerceptionPlanner::createPandaBundleContext() {
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(
      "robot_description_5link");
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_model->getJointModelGroup(getGroupName());
  planning_scene_monitor::PlanningSceneMonitorPtr psm;
  psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      robot_model_loader);
  psm->publishDebugInformation(true);
  psm->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::
                                     DEFAULT_COLLISION_OBJECT_TOPIC,
                                 planning_scene_monitor::PlanningSceneMonitor::
                                     DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                 false /* skip octomap monitor */);

  pandaBundleContext_ = std::make_shared<MyMoveitContext>(psm, robot_model);
  moveit::core::RobotStatePtr ready_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
  ready_state->setToDefaultValues(joint_model_group, "ready");
  psm->updateSceneWithCurrentState();

  planning_interface::MotionPlanRequest base_req;
  planning_interface::MotionPlanRequest bundle_req =
      context_->getMotionPlanRequest();

  // setting start state
  base_req.start_state.joint_state.header.stamp = ros::Time::now();
  base_req.start_state.joint_state.name = joint_model_group->getVariableNames();
  sensor_msgs::JointState bundle_state = bundle_req.start_state.joint_state;
  assert(bundle_state.position.size() >= 5);
  std::vector<double> start_joint_values{
      bundle_state.position[0], bundle_state.position[1],
      bundle_state.position[2], bundle_state.position[3],
      bundle_state.position[4]};
  base_req.start_state.joint_state.position = start_joint_values;
  std::fill(std::begin(start_joint_values), std::end(start_joint_values), 0);
  base_req.start_state.joint_state.velocity = start_joint_values;
  base_req.start_state.joint_state.effort = start_joint_values;

  // setting goal state
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
  moveit::core::RobotState goal_state(*robot_state);

  // the QRRT planner will do this internally
  assert(joint_goal_pos_.size() >= 5);
  std::vector<double> joint_goal_pos =
      tacbot::utilities::slice(joint_goal_pos_, 0, 4);
  goal_state.setJointGroupPositions(joint_model_group, joint_goal_pos);
  double tolerance = 0.001;
  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(
          goal_state, joint_model_group, tolerance);
  base_req.goal_constraints.push_back(joint_goal);

  // setting planning params
  base_req.group_name = bundle_req.group_name;
  base_req.allowed_planning_time = bundle_req.allowed_planning_time;
  base_req.planner_id = bundle_req.planner_id;
  base_req.max_acceleration_scaling_factor =
      bundle_req.max_acceleration_scaling_factor;
  base_req.max_velocity_scaling_factor = bundle_req.max_velocity_scaling_factor;

  pandaBundleContext_->createPlanningContext(base_req);
}

}  // namespace tacbot