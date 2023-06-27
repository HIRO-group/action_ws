#include "base_planner.h"

using namespace std::chrono;
constexpr char LOGNAME[] = "contact_planner";

namespace tacbot {

BasePlanner::BasePlanner() {
  joint_goal_pos_ = std::vector<double>{-1.0, 0.7, 0.7, -1.0, -0.7, 2.0, 0.0};
}

void BasePlanner::setCurToStartState(
    planning_interface::MotionPlanRequest& req) {
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  robot_state->setToDefaultValues(joint_model_group_, "ready");
  psm_->updateSceneWithCurrentState();

  req.start_state.joint_state.header.stamp = ros::Time::now();
  req.start_state.joint_state.name = joint_model_group_->getVariableNames();

  std::vector<double> start_joint_values;
  robot_state->copyJointGroupPositions(joint_model_group_, start_joint_values);
  req.start_state.joint_state.position = start_joint_values;

  start_joint_values.clear();
  robot_state->copyJointGroupVelocities(joint_model_group_, start_joint_values);
  req.start_state.joint_state.velocity = start_joint_values;

  start_joint_values.clear();
  robot_state->copyJointGroupAccelerations(joint_model_group_,
                                           start_joint_values);
  req.start_state.joint_state.effort = start_joint_values;
}

void BasePlanner::setStartState(planning_interface::MotionPlanRequest& req,
                                const std::vector<double>& pos) {
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  // robot_state->setToDefaultValues(joint_model_group_, "ready");
  robot_state->setVariablePositions(pos);
  robot_state->update();
  ROS_INFO_NAMED(LOGNAME, "New state positions.");
  robot_state->printStatePositions(std::cout);

  // planning_scene_monitor::CurrentStateMonitorPtr csm =
  // psm_->getStateMonitor(); csm->setToCurrentState(*robot_state);
  // planning_scene_monitor::LockedPlanningSceneRW(psm_)->setCurrentState(
  //     *robot_state);

  psm_->updateSceneWithCurrentState();

  req.start_state.joint_state.header.stamp = ros::Time::now();
  req.start_state.joint_state.name = joint_model_group_->getVariableNames();

  std::vector<double> start_joint_values;
  robot_state->copyJointGroupPositions(joint_model_group_, start_joint_values);
  req.start_state.joint_state.position = start_joint_values;

  start_joint_values.clear();
  robot_state->copyJointGroupVelocities(joint_model_group_, start_joint_values);
  req.start_state.joint_state.velocity = start_joint_values;

  start_joint_values.clear();
  robot_state->copyJointGroupAccelerations(joint_model_group_,
                                           start_joint_values);
  req.start_state.joint_state.effort = start_joint_values;
}

moveit_msgs::Constraints BasePlanner::createJointGoal() {
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));

  moveit::core::RobotState goal_state(*robot_state);

  goal_state.setJointGroupPositions(joint_model_group_, joint_goal_pos_);

  double tolerance = 0.001;

  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(
          goal_state, joint_model_group_, tolerance);

  return joint_goal;
}

void BasePlanner::changePlanner() {
  ompl::geometric::SimpleSetupPtr simple_setup = context_->getOMPLSimpleSetup();
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();

  ompl::base::PlannerPtr planner;

  planner = std::make_shared<ompl::geometric::RRTConnect>(
      simple_setup->getSpaceInformation());

  simple_setup->setPlanner(planner);
}

bool BasePlanner::generatePlan(planning_interface::MotionPlanResponse& res) {
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  bool is_solved = context_->solve(res);
  if (is_solved) {
    std::size_t state_count = res.trajectory_->getWayPointCount();
    ROS_INFO_NAMED(LOGNAME, "Motion planner reported a solution path with %ld",
                   state_count);
  }

  if (is_solved && res.trajectory_) {
    trajectory_processing::TimeOptimalTrajectoryGeneration time_param_(
        0.05, 0.001, 0.01);  // 0.001 for real execution

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

void BasePlanner::init() {
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      "robot_description");

  ROS_INFO_NAMED(LOGNAME, "getModel");
  robot_model_ = robot_model_loader_->getModel();

  ROS_INFO_NAMED(LOGNAME, "getJointModelGroup");
  joint_model_group_ = robot_model_->getJointModelGroup(group_name_);

  dof_ = joint_model_group_->getActiveVariableCount();

  // std::shared_ptr<tf2_ros::Buffer> tf_buffer =
  //     std::make_shared<tf2_ros::Buffer>();

  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      robot_model_loader_);

  psm_->publishDebugInformation(true);

  ROS_INFO_NAMED(LOGNAME, "startSceneMonitor");
  psm_->startSceneMonitor();

  ROS_INFO_NAMED(LOGNAME, "startStateMonitor");
  psm_->startStateMonitor();

  ROS_INFO_NAMED(LOGNAME, "startWorldGeometryMonitor");
  psm_->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::
                                      DEFAULT_COLLISION_OBJECT_TOPIC,
                                  planning_scene_monitor::PlanningSceneMonitor::
                                      DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                  false /* skip octomap monitor */);

  ROS_INFO_NAMED(LOGNAME, "startPublishingPlanningScene");
  psm_->startPublishingPlanningScene(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  ROS_INFO_NAMED(LOGNAME, "providePlanningSceneService");
  psm_->providePlanningSceneService();

  ROS_INFO_NAMED(LOGNAME, "KinematicsMetrics");
  kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getRobotModel());

  ROS_INFO_NAMED(LOGNAME, "getStateMonitor");
  planning_scene_monitor::CurrentStateMonitorPtr csm = psm_->getStateMonitor();
  csm->enableCopyDynamics(true);

  ROS_INFO_NAMED(LOGNAME, "robot_state");
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));

  ROS_INFO_NAMED(LOGNAME, "updateSceneWithCurrentState");
  robot_state->setToDefaultValues(joint_model_group_, "ready");
  psm_->updateSceneWithCurrentState();

  robot_state_ = std::make_shared<moveit::core::RobotState>(*robot_state);

  ROS_INFO_NAMED(LOGNAME, "vis_data_");
  vis_data_ = std::make_shared<VisualizerData>();
  ROS_INFO_NAMED(LOGNAME, "init done");
}

std::string BasePlanner::getGroupName() { return group_name_; }

}  // namespace tacbot