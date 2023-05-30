#include "perception_planner.h"

#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/base/objectives/MinimizeContactObjective.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/multilevel/planners/qrrt/QRRTStar.h>

#include "ompl/geometric/planners/informedtrees/BITstar.h"
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
  setGoalState(1);
  setObstacleScene(2);
  sphericalCollisionPermission(true);
}

void PerceptionPlanner::setGoalState(std::size_t option) {
  switch (option) {
    default:
    case 1:
      joint_goal_pos_ =
          std::vector<double>{-1.0, 0.7, 0.7, -1.0, -0.7, 2.0, 0.0};
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
    default:
      ROS_ERROR_NAMED(LOGNAME,
                      "Selected obstacle scene option is out of bounds: %ld",
                      option);
      break;
  }

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
  ROS_INFO_NAMED(LOGNAME, "context_->getOMPLSimpleSetup()");
  ompl::geometric::SimpleSetupPtr simple_setup = context_->getOMPLSimpleSetup();

  ROS_INFO_NAMED(LOGNAME, "simple_setup->getSpaceInformation()");
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();

  ompl::base::PlannerPtr planner;

  // Why is this not working? TODO(nn) investigate this planner
  ROS_INFO_NAMED(LOGNAME, "getLOPandaSpace");
  ompl::base::SpaceInformationPtr si2 = getLOPandaSpace();

  std::vector<ompl::base::SpaceInformationPtr> siVec;

  siVec.emplace_back(si);
  siVec.emplace_back(si2);

  ROS_INFO_NAMED(LOGNAME, "ompl::multilevel::QRRTStar");
  planner = std::make_shared<ompl::multilevel::QRRTStar>(si);

  // planner = std::make_shared<ompl::geometric::BITstar>(si);

  std::function<double(const ompl::base::State*)> optFunc;

  optFunc = std::bind(&PerceptionPlanner::overlapMagnitude, this,
                      std::placeholders::_1);
  optimization_objective_ =
      std::make_shared<ompl::base::MinimizeContactObjective>(si, optFunc);
  simple_setup->setOptimizationObjective(optimization_objective_);

  optimization_objective_->setCostToGoHeuristic(
      &ompl::base::goalRegionCostToGo);

  simple_setup->setPlanner(planner);
}

double PerceptionPlanner::overlapMagnitude(
    const ompl::base::State* base_state) {
  sphericalCollisionPermission(false);

  const ompl::base::RealVectorStateSpace::StateType& vec_state =
      *base_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles = utilities::toStlVec(vec_state, dof_);
  moveit::core::RobotState robot_state(*robot_state_);
  robot_state.setJointGroupPositions(joint_model_group_, joint_angles);

  double contact_depth = getContactDepth(robot_state);
  sphericalCollisionPermission(true);
  return contact_depth;
}

void PerceptionPlanner::sphericalCollisionPermission(bool is_allowed) {
  // collision_detection::CollisionEnvConstPtr col_env =
  //     planning_scene_monitor::LockedPlanningSceneRW(psm_)->getCollisionEnv();

  // ROS_INFO_NAMED(LOGNAME, "sphericalCollisionPermission");

  collision_detection::AllowedCollisionMatrix& acm =
      planning_scene_monitor::LockedPlanningSceneRW(psm_)
          ->getAllowedCollisionMatrixNonConst();
  // acm.print(std::cout);

  std::vector<std::string> sphere_names;
  for (auto obstacle : obstacles_) {
    sphere_names.emplace_back(obstacle.name);
  }

  for (auto name : sphere_names) {
    acm.setEntry(name, is_allowed);
  }

  // planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
  // context_->setPlanningScene(lscene);

  // acm.print(std::cout);
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
}

double PerceptionPlanner::getContactDepth(
    moveit::core::RobotState robot_state) {
  // ROS_INFO_NAMED(LOGNAME, "getContactDepth");

  collision_detection::CollisionRequest collision_request;
  collision_request.distance = false;
  collision_request.cost = false;
  collision_request.contacts = true;
  collision_request.max_contacts = 20;
  collision_request.max_contacts_per_pair = 2;
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
    // ROS_INFO_NAMED(LOGNAME, "Number of subcontacts: %ld", num_subcontacts);
    for (std::size_t subc_idx = 0; subc_idx < num_subcontacts; subc_idx++) {
      collision_detection::Contact subcontact = contact.second[subc_idx];
      // ROS_INFO_NAMED(LOGNAME, "Body 1: %s", subcontact.body_name_1.c_str());
      // ROS_INFO_NAMED(LOGNAME, "Body 2: %s", subcontact.body_name_2.c_str());
      // ROS_INFO_NAMED(LOGNAME, "Depth: %f", subcontact.depth);

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

bool PerceptionPlanner::generatePlan(
    planning_interface::MotionPlanResponse& res) {
  ROS_INFO_NAMED(LOGNAME, "Generating a plan with the perception planner.");

  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  bool is_solved = context_->solve(res);

  // remove the already queried status from here when doing path simplificationz
  // moveit_planners/ompl/ompl_interface/src/detail/state_validity_checker.cpp

  if (is_solved) {
    std::size_t state_count = res.trajectory_->getWayPointCount();
    ROS_INFO_NAMED(LOGNAME, "Motion planner reported a solution path with %ld",
                   state_count);
  }

  if (is_solved && res.trajectory_) {
    trajectory_processing::TimeOptimalTrajectoryGeneration time_param_(
        0.05, 0.001, 0.01);

    moveit::core::RobotStatePtr first_prt =
        res.trajectory_->getFirstWayPointPtr();
    moveit::core::RobotStatePtr last_prt =
        res.trajectory_->getLastWayPointPtr();
    first_prt->setVariableVelocities(std::vector<double>{0, 0, 0, 0, 0, 0, 0});
    last_prt->setVariableVelocities(std::vector<double>{0, 0, 0, 0, 0, 0, 0});

    ROS_INFO_NAMED(LOGNAME, "Computing time parameterization.");
    planning_interface::MotionPlanRequest req =
        context_->getMotionPlanRequest();
    ROS_INFO_NAMED(LOGNAME, "req.max_velocity_scaling_factor %f",
                   req.max_velocity_scaling_factor);
    ROS_INFO_NAMED(LOGNAME, "req.max_acceleration_scaling_factor %f",
                   req.max_acceleration_scaling_factor);
    if (!time_param_.computeTimeStamps(*res.trajectory_,
                                       req.max_velocity_scaling_factor,
                                       req.max_acceleration_scaling_factor)) {
      ROS_ERROR_NAMED(LOGNAME,
                      "Time parametrization for the solution path failed.");
      is_solved = false;
    } else {
      ROS_INFO_NAMED(LOGNAME, "Time parameterization success.");
      plan_response_ = res;
    }
  }

  ROS_INFO_NAMED(LOGNAME, "Is plan generated? %d", is_solved);
  return is_solved;
}

ompl::base::SpaceInformationPtr PerceptionPlanner::getLOPandaSpace() {
  robot_model_loader::RobotModelLoaderPtr robot_model_loader;
  robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(
      "robot_description_5link");
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  planning_scene_monitor::PlanningSceneMonitorPtr psm;

  psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      robot_model_loader);

  psm->publishDebugInformation(true);
  ROS_INFO_NAMED(LOGNAME, "startWorldGeometryMonitor");
  psm->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::
                                     DEFAULT_COLLISION_OBJECT_TOPIC,
                                 planning_scene_monitor::PlanningSceneMonitor::
                                     DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                 false /* skip octomap monitor */);

  std::string group_name = "panda_arm";
  ROS_INFO_NAMED(LOGNAME, "space_spec");
  ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model,
                                                               group_name);

  ROS_INFO_NAMED(LOGNAME, "JointModelStateSpace");

  ompl_interface::ModelBasedStateSpacePtr state_space =
      std::make_shared<ompl_interface::JointModelStateSpace>(space_spec);

  ROS_INFO_NAMED(LOGNAME, "computeLocations");

  state_space->computeLocations();

  ROS_INFO_NAMED(LOGNAME, "ompl_interface");

  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface =
      std::make_unique<ompl_interface::OMPLInterface>(robot_model, nh_);

  ROS_INFO_NAMED(LOGNAME, "pconfig_map");
  planning_interface::PlannerConfigurationMap pconfig_map =
      ompl_interface->getPlannerConfigurations();

  std::string planner_id = "panda_arm[RRTConnect]";

  ROS_INFO_NAMED(LOGNAME, "pconfig_map.end()");
  auto pc = pconfig_map.end();
  pc = pconfig_map.find(planner_id);

  if (pc == pconfig_map.end()) {
    ROS_ERROR_NAMED(LOGNAME,
                    "Cannot find planning configuration for planner %s ",
                    planner_id.c_str());
  } else {
    ROS_INFO_NAMED(LOGNAME, "Found planning configuration for planner %s.",
                   planner_id.c_str());
  }

  planning_interface::PlannerConfigurationSettings pconfig_settings =
      pc->second;

  ROS_INFO_NAMED(LOGNAME, "planning_context_manager");

  ompl_interface::PlanningContextManager planning_context_manager =
      ompl_interface->getPlanningContextManager();

  ROS_INFO_NAMED(LOGNAME, "context_spec");
  ompl_interface::ModelBasedPlanningContextSpecification context_spec;
  context_spec.config_ = pconfig_settings.config;
  ROS_INFO_NAMED(LOGNAME, "getPlannerSelector");

  context_spec.planner_selector_ =
      planning_context_manager.getPlannerSelector();
  context_spec.constraint_sampler_manager_ =
      std::make_shared<constraint_samplers::ConstraintSamplerManager>();
  context_spec.state_space_ = state_space;

  ROS_INFO_NAMED(LOGNAME, "ompl_simple_setup");

  ompl::geometric::SimpleSetupPtr ompl_simple_setup =
      std::make_shared<ompl::geometric::SimpleSetup>(state_space);
  context_spec.ompl_simple_setup_ = ompl_simple_setup;

  ROS_INFO_NAMED(LOGNAME, "ModelBasedPlanningContext");

  ompl_interface::ModelBasedPlanningContextPtr context =
      std::make_shared<ompl_interface::ModelBasedPlanningContext>(group_name,
                                                                  context_spec);

  ROS_INFO_NAMED(LOGNAME, "setStateValidityChecker");

  ompl_simple_setup->setStateValidityChecker(
      ompl::base::StateValidityCheckerPtr(
          new ompl_interface::StateValidityChecker(context.get())));
  ROS_INFO_NAMED(LOGNAME, "spaceinfo");

  ompl::base::SpaceInformationPtr si = ompl_simple_setup->getSpaceInformation();
  return si;
}

}  // namespace tacbot