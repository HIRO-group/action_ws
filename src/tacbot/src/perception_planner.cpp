#include "perception_planner.h"

#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <ompl/base/objectives/MinimizeContactObjective.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/multilevel/planners/qrrt/QRRTStar.h>

#include "ompl/geometric/planners/informedtrees/BITstar.h"

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
  setObstacleScene(1);
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
  spherical_obstacles_.clear();
  sim_obstacle_pos_.clear();
  ROS_INFO_NAMED(LOGNAME, "Obstacle scene option selected: %ld", option);
  switch (option) {
    case 1:
      spherical_obstacles_.emplace_back(
          std::make_pair(Eigen::Vector3d{0.5, 0.0, 0.6}, 0.05));
      spherical_obstacles_.emplace_back(
          std::make_pair(Eigen::Vector3d{0.45, -0.4, 0.6}, 0.1));
      break;
    default:
      ROS_ERROR_NAMED(LOGNAME,
                      "Selected obstacle scene option is out of bounds: %ld",
                      option);
      break;
  }

  for (auto obstacle : spherical_obstacles_) {
    addSphericalObstacle(obstacle.first, obstacle.second);
  }

  for (auto sphere : spherical_obstacles_) {
    contact_perception_->addSphere(sphere.first, sphere.second);
  }
}

void PerceptionPlanner::addSphericalObstacle(const Eigen::Vector3d& center,
                                             double radius) {
  const double inc = 0.4;
  for (double theta = 0; theta < 2 * M_PI; theta += inc) {
    for (double phi = 0; phi < 2 * M_PI; phi += inc) {
      double x = radius * cos(phi) * sin(theta) + center[0];
      double y = radius * sin(phi) * sin(theta) + center[1];
      double z = radius * cos(theta) + center[2];
      sim_obstacle_pos_.emplace_back(Eigen::Vector3d{x, y, z});
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
  //  ROS_INFO_NAMED(LOGNAME,
  //  "std::make_shared<ompl::multilevel::QRRTStar>(si)");
  //  std::shared_ptr<ompl::multilevel::QRRTStar>
  //  planner = std::make_shared<ompl::multilevel::QRRTStar>(si);

  ROS_INFO_NAMED(LOGNAME, "BITstar planner");
  planner = std::make_shared<ompl::geometric::ABITstar>(si);

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
  ROS_INFO_NAMED(LOGNAME, "overlapMagnitude");
  sphericalCollisionPermission(false);

  const ompl::base::RealVectorStateSpace::StateType& vec_state =
      *base_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles = utilities::toStlVec(vec_state, dof_);
  moveit::core::RobotState robot_state(*robot_state_);
  robot_state.setJointGroupPositions(joint_model_group_, joint_angles);

  sphericalCollisionPermission(true);

  return getContactDepth(robot_state);
}

void PerceptionPlanner::sphericalCollisionPermission(bool is_allowed) {
  // collision_detection::CollisionEnvConstPtr col_env =
  //     planning_scene_monitor::LockedPlanningSceneRW(psm_)->getCollisionEnv();

  collision_detection::AllowedCollisionMatrix& acm =
      planning_scene_monitor::LockedPlanningSceneRW(psm_)
          ->getAllowedCollisionMatrixNonConst();
  // acm.print(std::cout);

  std::vector<std::string> sphere_names = contact_perception_->getSphereNames();

  for (auto name : sphere_names) {
    acm.setEntry(name, is_allowed);
  }
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
  ROS_INFO_NAMED(LOGNAME, "getContactDepth");

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
  ROS_INFO_NAMED(LOGNAME, "collision: %d", collision);

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
      ROS_INFO_NAMED(LOGNAME, "Body 1: %s", subcontact.body_name_1.c_str());
      ROS_INFO_NAMED(LOGNAME, "Body 2: %s", subcontact.body_name_2.c_str());
      ROS_INFO_NAMED(LOGNAME, "Depth: %f", subcontact.depth);
      total_depth += std::abs(subcontact.depth);
      ROS_INFO_NAMED(LOGNAME, "Total Depth: %f", total_depth);
    }
  }

  return total_depth;
}

bool PerceptionPlanner::generatePlan(
    planning_interface::MotionPlanResponse& res) {
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  bool is_solved = context_->solve(res);

  sphericalCollisionPermission(false);
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
  context_->setPlanningScene(lscene);
  double timeout = 10;
  context_->simplifySolution(timeout);

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

}  // namespace tacbot