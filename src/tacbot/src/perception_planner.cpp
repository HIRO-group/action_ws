#include "perception_planner.h"

#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/base/objectives/MinimizeContactObjective.h>
#include <ompl/base/objectives/VFMagnitudeOptimizationObjective.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/ProjectionTypes.h>
#include <ompl/multilevel/datastructures/projections/ModelBased_RN_RM.h>
#include <ompl/multilevel/planners/qmp/QMPStar.h>
#include <ompl/multilevel/planners/qrrt/QRRTStar.h>

#include <nlopt.hpp>

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

  if (planner_name_ == "BITstar") {
    std::function<double(const ompl::base::State*)> optFunc;

    optFunc = std::bind(&PerceptionPlanner::overlapMagnitude, this,
                        std::placeholders::_1);
    optimization_objective_ =
        std::make_shared<ompl::base::MinimizeContactObjective>(si, optFunc);
    simple_setup->setOptimizationObjective(optimization_objective_);

    planner = std::make_shared<ompl::geometric::ABITstar>(si);

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
    vFieldFuncDuo = std::bind(&PerceptionPlanner::obstacleFieldDuo, this,
                              std::placeholders::_1, std::placeholders::_2);

    std::function<Eigen::VectorXd(const ompl::base::State*)> vFieldFunc;
    vFieldFunc = std::bind(&PerceptionPlanner::obstacleField, this,
                           std::placeholders::_1);

    optimization_objective_ =
        std::make_shared<ompl::base::VFMagnitudeOptimizationObjective>(
            si, vFieldFunc);

    simple_setup->setOptimizationObjective(optimization_objective_);
    planner = std::make_shared<ompl::geometric::ContactTRRT>(si, vFieldFuncDuo);

  } else {
    ROS_ERROR_NAMED(LOGNAME, "The following planner is not supported: %s",
                    planner_name_.c_str());
    throw std::invalid_argument(planner_name_);
  }

  optimization_objective_->setCostToGoHeuristic(
      &ompl::base::goalRegionCostToGo);
  simple_setup->setPlanner(planner);
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

double OptimizationProblem::objective(const std::vector<double>& x,
                                      std::vector<double>& grad, void* data) {
  OptimizationProblem* problem = static_cast<OptimizationProblem*>(data);

  double result = problem->a * x[0] * x[0] + problem->b;
  if (!grad.empty()) {
    grad[0] = 2 * problem->a * x[0];
  }
  return result;
}

double OptimizationProblem::constraint(const std::vector<double>& x,
                                       std::vector<double>& grad, void* data) {
  OptimizationProblem* problem = static_cast<OptimizationProblem*>(data);
  double a = problem->a;
  double b = problem->b;
  return -1;
}

double OptimizationProblem::optimize() {
  nlopt::opt opt(nlopt::LN_COBYLA, 1);

  opt.set_min_objective(
      [](const std::vector<double>& x, std::vector<double>& grad,
         void* data) -> double {
        return static_cast<OptimizationProblem*>(data)->objective(x, grad,
                                                                  data);
      },
      this);

  // Set the constraint function
  // opt.add_inequality_constraint(
  //     [](const std::vector<double>& x, std::vector<double>& grad,
  //        void* data) -> double {
  //       return static_cast<OptimizationProblem*>(data)->constraint(x, grad,
  //                                                                  data);
  //     },
  //     this, 1e-8);

  std::vector<double> lb = {-10};
  std::vector<double> ub = {10};
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);

  opt.set_xtol_rel(1e-4);

  std::vector<double> x = {2};
  double minf;
  nlopt::result result = opt.optimize(x, minf);

  if (result == nlopt::SUCCESS) {
    std::cout << "Optimization success! Minimum value found: " << minf
              << std::endl;
  } else {
    std::cout << "Optimization failed!" << std::endl;
  }

  return minf;
}

}  // namespace tacbot