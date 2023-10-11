#include "base_planner.h"

#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

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
  // ROS_INFO_NAMED(LOGNAME, "New state positions.");
  // robot_state->printStatePositions(std::cout);

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
  return createJointGoal(joint_goal_pos_);
}

moveit_msgs::Constraints BasePlanner::createJointGoal(
    std::vector<double> joint_goal_pos) {
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));

  moveit::core::RobotState goal_state(*robot_state);

  goal_state.setJointGroupPositions(joint_model_group_, joint_goal_pos);

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
  if (context_->solve(res)) {
    std::size_t state_count = res.trajectory_->getWayPointCount();
    ROS_INFO_NAMED(LOGNAME, "State count in solution path %ld", state_count);
    plan_response_ = res;
  } else {
    ROS_ERROR_NAMED(LOGNAME, "Failed to find motion plan.");
    return false;
  }
  return true;
}

bool BasePlanner::parameterizePlan(
    planning_interface::MotionPlanResponse& res) {
  if (res.error_code_.val != res.error_code_.SUCCESS || !res.trajectory_) {
    ROS_ERROR_NAMED(LOGNAME, "Invalid solution. Cannot parameterize.");
    return false;
  }

  ROS_INFO_NAMED(LOGNAME, "Computing time parameterization.");
  trajectory_processing::TimeOptimalTrajectoryGeneration time_param_(
      0.05, 0.001, 0.01);
  planning_interface::MotionPlanRequest req = context_->getMotionPlanRequest();

  moveit::core::RobotStatePtr first_prt =
      res.trajectory_->getFirstWayPointPtr();
  moveit::core::RobotStatePtr last_prt = res.trajectory_->getLastWayPointPtr();
  first_prt->setVariableVelocities(std::vector<double>{0, 0, 0, 0, 0, 0, 0});
  last_prt->setVariableVelocities(std::vector<double>{0, 0, 0, 0, 0, 0, 0});

  if (time_param_.computeTimeStamps(*res.trajectory_,
                                    req.max_velocity_scaling_factor,
                                    req.max_acceleration_scaling_factor)) {
    ROS_INFO_NAMED(LOGNAME, "Time parameterization success.");
  } else {
    ROS_ERROR_NAMED(LOGNAME, "Time parametrization for path failed.");
    return false;
  }

  // if (trajectory_processing::RuckigSmoothing::applySmoothing(
  //         *res.trajectory_, req.max_velocity_scaling_factor,
  //         req.max_acceleration_scaling_factor)) {
  //   ROS_INFO_NAMED(LOGNAME, "Ruckig smoothing for the solution success.");
  // } else {
  //   ROS_ERROR_NAMED(LOGNAME, "Ruckig smoothing for the solution failure.");
  //   return false;
  // }

  plan_response_ = res;
  return true;
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

bool BasePlanner::solveFK(std::vector<double> joint_values) {
  const kinematics::KinematicsBaseConstPtr ik_solver =
      joint_model_group_->getSolverInstance();

  std::vector<std::string> link_names;
  link_names.emplace_back("panda_link8");
  std::vector<geometry_msgs::Pose> poses;

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  robot_state->setToDefaultValues(joint_model_group_, "ready");

  robot_state->copyJointGroupPositions(joint_model_group_, joint_values);
  bool status = ik_solver->getPositionFK(link_names, joint_values, poses);

  std::cout << "fk status: " << status << std::endl;

  for (std::size_t i = 0; i < poses.size(); i++) {
    geometry_msgs::Pose pose = poses[i];
    std::cout << "pose.position.x: " << pose.position.x << std::endl;
    std::cout << "pose.position.y: " << pose.position.y << std::endl;
    std::cout << "pose.position.z: " << pose.position.z << std::endl;
    std::cout << "pose.orientation.x: " << pose.orientation.x << std::endl;
    std::cout << "pose.orientation.y: " << pose.orientation.y << std::endl;
    std::cout << "pose.orientation.z: " << pose.orientation.z << std::endl;
    std::cout << "pose.orientation.w: " << pose.orientation.w << std::endl;
  }
  return status;
}

bool BasePlanner::solveIK(const geometry_msgs::Pose& ik_pose,
                          const std::vector<double>& ik_seed_state,
                          std::vector<double>& solution) {
  const kinematics::KinematicsBaseConstPtr ik_solver =
      joint_model_group_->getSolverInstance();

  // std::vector<std::string> link_names;
  // link_names.emplace_back("panda_link8");
  // std::vector<geometry_msgs::Pose> poses;
  // std::vector<double> start_joint_values;

  // moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
  //     planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  // robot_state->setToDefaultValues(joint_model_group_, "ready");

  // robot_state->copyJointGroupPositions(joint_model_group_,
  // start_joint_values); bool status = ik_solver->getPositionFK(link_names,
  // start_joint_values, poses);

  // std::cout << "fk status: " << status << std::endl;

  // for (std::size_t i = 0; i < poses.size(); i++) {
  //   geometry_msgs::Pose pose = poses[i];
  //   std::cout << "pose.position.x: " << pose.position.x << std::endl;
  //   std::cout << "pose.position.y: " << pose.position.y << std::endl;
  //   std::cout << "pose.position.z: " << pose.position.z << std::endl;
  //   std::cout << "pose.orientation.x: " << pose.orientation.x << std::endl;
  //   std::cout << "pose.orientation.y: " << pose.orientation.y << std::endl;
  //   std::cout << "pose.orientation.z: " << pose.orientation.z << std::endl;
  //   std::cout << "pose.orientation.w: " << pose.orientation.w << std::endl;
  // }

  moveit_msgs::MoveItErrorCodes error_code;
  bool is_valid =
      ik_solver->getPositionIK(ik_pose, ik_seed_state, solution, error_code);

  if (error_code.val != error_code.SUCCESS) {
    ROS_ERROR_STREAM_NAMED(
        LOGNAME,
        "IK solution callback failed with with error code: " << error_code.val);
    return false;
  }

  return true;
}

std::string BasePlanner::getGroupName() { return group_name_; }

bool BasePlanner::calculateEEPath() {
  moveit_msgs::MotionPlanResponse msg;
  plan_response_.getMessage(msg);
  std::size_t num_pts = msg.trajectory.joint_trajectory.points.size();
  ROS_INFO_NAMED(LOGNAME, "Trajectory num_pts: %ld", num_pts);

  vis_data_->ee_path_pts_.clear();

  for (std::size_t pt_idx = 0; pt_idx < num_pts; pt_idx++) {
    // ROS_INFO_NAMED(LOGNAME, "Analyzing trajectory point number: %ld",
    // pt_idx);

    trajectory_msgs::JointTrajectoryPoint point =
        msg.trajectory.joint_trajectory.points[pt_idx];

    std::vector<double> joint_angles(dof_, 0.0);
    for (std::size_t jnt_idx = 0; jnt_idx < dof_; jnt_idx++) {
      joint_angles[jnt_idx] = point.positions[jnt_idx];
    }

    moveit::core::RobotState robot_state(
        psm_->getPlanningScene()->getRobotModel());
    robot_state.setJointGroupPositions(joint_model_group_, joint_angles);
    robot_state.update();

    std::vector<std::string> tips;
    joint_model_group_->getEndEffectorTips(tips);
    Eigen::Isometry3d tip_tf =
        robot_state.getGlobalLinkTransform("panda_link8");
    Eigen::Vector3d tip_pos{tip_tf.translation().x(), tip_tf.translation().y(),
                            tip_tf.translation().z()};
    vis_data_->ee_path_pts_.emplace_back(tip_pos);
  }
  return true;
}

}  // namespace tacbot