
// ROS
#include <ros/ros.h>

// C++
#include <algorithm>
#include <valarray>
#include <vector>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/ompl_interface/detail/constrained_goal_sampler.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>

#include "ompl/geometric/planners/est/EST.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

constexpr char LOGNAME[] = "pick_and_place_node";

// ======================= SHARED VARIABLES
// ==========================================
const Eigen::Vector3d obstacle_pos_(0.4, 0.0, 0.5);
const std::vector<double> joint_goal_pos_ = {-1.0, 0.7, 0.7, -1.0,
                                             -0.7, 2.0, 0.0};

const std::string group_name_ = "panda_arm";

const moveit::core::JointModelGroup* joint_model_group_;
robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
planning_scene_monitor::PlanningSceneMonitorPtr psm_;
moveit::core::RobotModelPtr robot_model_;
kinematics_metrics::KinematicsMetricsPtr kinematics_metrics_;
moveit::core::RobotStatePtr robot_state_;

std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

ros::Publisher marker_pub_;
ros::Publisher arrow_pub_;
ros::Publisher trajectory_publisher_;
ros::Publisher rep_state_publisher_;

std::size_t dof_ = 7;  // get this from robot model

std::vector<std::vector<double>> sample_joint_angles_;
std::vector<std::vector<double>> sample_desired_angles_;

std::size_t viz_state_idx_ = 0;
std::size_t sample_state_count_ = 0;
std::vector<Eigen::VectorXd> repulsed_vec_at_link_;
std::vector<Eigen::VectorXd> repulsed_origin_at_link_;

// ==========================================
// ==========================================

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_,
                          bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ =
      svd.singularValues();
  Eigen::MatrixXd S_ =
      M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) =
        (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() *
                            svd.matrixU().transpose());
}

void printPlannerConfigMap(
    const planning_interface::PlannerConfigurationMap& planner_config_map) {
  for (auto config : planner_config_map) {
    ROS_INFO_NAMED(LOGNAME, "Map Name: %s", config.first.c_str());
    ROS_INFO_NAMED(LOGNAME, "\tGroup: %s", config.second.group.c_str());
    ROS_INFO_NAMED(LOGNAME, "\tName: %s", config.second.name.c_str());

    for (auto setting : config.second.config) {
      ROS_INFO_NAMED(LOGNAME, "\t\tSetting: %s", setting.first.c_str());
      ROS_INFO_NAMED(LOGNAME, "\t\tValue: %s", setting.second.c_str());
    }
  }
}

void printJointTrajectory(
    const trajectory_msgs::JointTrajectory& joint_trajectory) {
  ROS_INFO_NAMED(LOGNAME, "Num joints: %ld",
                 joint_trajectory.joint_names.size());
  ROS_INFO_NAMED(LOGNAME, "Num points: %ld", joint_trajectory.points.size());

  for (int i = 0; i < joint_trajectory.joint_names.size(); i++) {
    std::string name = joint_trajectory.joint_names[i];
    std::cout << name << " ";
  }

  for (int i = 0; i < joint_trajectory.points.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint traj_point =
        joint_trajectory.points[i];
    std::cout << "\npositions" << std::endl;
    for (int k = 0; k < traj_point.positions.size(); k++) {
      double position = traj_point.positions[k];
      std::cout << position << " ";
    }

    std::cout << "\nvelocities" << std::endl;
    for (int k = 0; k < traj_point.velocities.size(); k++) {
      double velocity = traj_point.velocities[k];
      std::cout << velocity << " ";
    }

    std::cout << "\naccelerations" << std::endl;
    for (int k = 0; k < traj_point.accelerations.size(); k++) {
      double accel = traj_point.accelerations[k];
      std::cout << accel << " ";
    }

    std::cout << "\neffort" << std::endl;
    for (int k = 0; k < traj_point.effort.size(); k++) {
      double effort = traj_point.effort[k];
      std::cout << effort << " ";
    }
  }
}

void visualizeRepulseVec(std::size_t state_num) {
  visualization_msgs::MarkerArray marker_array;
  for (std::size_t link_num = 0; link_num < dof_; link_num++) {
    auto repulsed_origin_at_link = repulsed_origin_at_link_[state_num];
    auto repulsed_vec_at_link = repulsed_vec_at_link_[state_num];

    std::valarray<double> origin = {repulsed_origin_at_link[link_num * 3],
                                    repulsed_origin_at_link[link_num * 3 + 1],
                                    repulsed_origin_at_link[link_num * 3 + 2]};

    std::valarray<double> dir = {repulsed_vec_at_link[link_num * 3],
                                 repulsed_vec_at_link[link_num * 3 + 1],
                                 repulsed_vec_at_link[link_num * 3 + 2]};

    std::valarray<double> vec = origin + dir;

    uint32_t shape = visualization_msgs::Marker::ARROW;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    // marker.ns = "basic_shapes";
    marker.id = link_num;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and DELETEALL
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point start_pt;
    start_pt.x = origin[0];
    start_pt.y = origin[1];
    start_pt.z = origin[2];

    geometry_msgs::Point end_pt;
    end_pt.x = vec[0];
    end_pt.y = vec[1];
    end_pt.z = vec[2];

    marker.points.push_back(start_pt);
    marker.points.push_back(end_pt);

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker_array.markers.push_back(marker);
  }
  arrow_pub_.publish(marker_array);
}

visualization_msgs::Marker getObstacleMarker() {
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  // marker.ns = "basic_shapes";
  marker.id = 10;
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and DELETEALL
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = obstacle_pos_[0];
  marker.pose.position.y = obstacle_pos_[1];
  marker.pose.position.z = obstacle_pos_[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  return marker;
}

void visualizeRepulseOrigin(std::size_t state_num) {
  std::vector<double> joint_angles = sample_joint_angles_[state_num];
  moveit::core::RobotStatePtr robot_state =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state->setJointGroupPositions(joint_model_group_, joint_angles);

  std::vector<std::string> link_names = joint_model_group_->getLinkModelNames();
  Eigen::Isometry3d joint_origin_tf = Eigen::Isometry3d::Identity();
  visualization_msgs::MarkerArray marker_array;
  for (std::size_t i = 0; i < dof_; i++) {
    const moveit::core::LinkModel* link_model =
        robot_state->getLinkModel(link_names[i]);
    Eigen::Isometry3d joint_origin_tf =
        robot_state->getGlobalLinkTransform(link_model);

    auto translation = joint_origin_tf.translation();
    // std::cout << "translation for " << link_names[i] << "\n"
    //           << translation << std::endl;

    uint32_t shape = visualization_msgs::Marker::SPHERE;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    // marker.ns = "basic_shapes";
    marker.id = i;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and DELETEALL
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = (double)translation.x();
    marker.pose.position.y = (double)translation.y();
    marker.pose.position.z = (double)translation.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_array.markers.push_back(marker);
  }

  visualization_msgs::Marker marker = getObstacleMarker();
  marker_array.markers.push_back(marker);
  marker_pub_.publish(marker_array);
}

void visualizeTrajectory(const planning_interface::MotionPlanResponse& res) {
  visual_tools_->deleteAllMarkers();
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  trajectory_publisher_.publish(display_trajectory);
  visual_tools_->publishTrajectoryLine(display_trajectory.trajectory.back(),
                                       joint_model_group_);
  visual_tools_->trigger();
}

void visualizeRepulsedState() {
  std::string user_input = " ";

  while (user_input != "q") {
    std::cout << "Displaying repulsed state with idx: " << viz_state_idx_
              << std::endl;

    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::MotionPlanResponse response;

    sensor_msgs::JointState joint_start_state;
    std::vector<std::string> names =
        joint_model_group_->getActiveJointModelNames();

    std::size_t num_joints = names.size();
    std::vector<double> joint_angles = sample_joint_angles_[viz_state_idx_];
    joint_start_state.name = names;
    joint_start_state.position = joint_angles;
    joint_start_state.velocity = std::vector<double>(num_joints, 0.0);
    joint_start_state.effort = std::vector<double>(num_joints, 0.0);

    moveit_msgs::RobotState robot_start_state;
    robot_start_state.joint_state = joint_start_state;

    response.group_name = group_name_;
    response.trajectory_start = robot_start_state;

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = names;

    trajectory_msgs::JointTrajectoryPoint point;

    joint_angles = sample_joint_angles_[viz_state_idx_];
    point.positions = joint_angles;
    point.velocities = std::vector<double>(num_joints, 0.0);
    point.accelerations = std::vector<double>(num_joints, 0.0);
    point.effort = std::vector<double>(num_joints, 0.0);
    point.time_from_start = ros::Duration(0.1);
    joint_trajectory.points.push_back(point);

    joint_angles = sample_desired_angles_[viz_state_idx_];
    point.positions = joint_angles;
    point.velocities = std::vector<double>(num_joints, 0.0);
    point.accelerations = std::vector<double>(num_joints, 0.0);
    point.effort = std::vector<double>(num_joints, 0.0);
    point.time_from_start = ros::Duration(1.5);
    joint_trajectory.points.push_back(point);

    moveit_msgs::RobotTrajectory robot_trajectory;
    robot_trajectory.joint_trajectory = joint_trajectory;

    response.trajectory = robot_trajectory;

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    rep_state_publisher_.publish(display_trajectory);

    visualizeRepulseVec(viz_state_idx_);
    visualizeRepulseOrigin(viz_state_idx_);

    viz_state_idx_++;

    std::cout << "Press 'q' to exit this visualization or 'c' to go to the "
                 "next state "
              << std::endl;
    std::cin >> user_input;
  }
}

void promptAnyInput() {
  std::cout << std::endl;
  std::cout << "Press any key to continue: ";
  getchar();
}

void setCurToStartState(
    planning_interface::MotionPlanRequest& req,
    const moveit::core::RobotStatePtr robot_state,
    const moveit::core::JointModelGroup* joint_model_group) {
  req.start_state.joint_state.header.stamp = ros::Time::now();
  req.start_state.joint_state.name = joint_model_group->getVariableNames();

  std::vector<double> start_joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, start_joint_values);
  req.start_state.joint_state.position = start_joint_values;

  start_joint_values.clear();
  robot_state->copyJointGroupVelocities(joint_model_group, start_joint_values);
  req.start_state.joint_state.velocity = start_joint_values;

  start_joint_values.clear();
  robot_state->copyJointGroupAccelerations(joint_model_group,
                                           start_joint_values);
  req.start_state.joint_state.effort = start_joint_values;
}

std::unique_ptr<ompl_interface::OMPLInterface> getOMPLInterface(
    const moveit::core::RobotModelConstPtr& model, const ros::NodeHandle& nh) {
  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface =
      std::make_unique<ompl_interface::OMPLInterface>(model, nh);
  return ompl_interface;
}

void setPlanningContextParams(
    ompl_interface::ModelBasedPlanningContextPtr& context) {
  unsigned int max_goal_samples_ = 10;
  unsigned int max_state_sampling_attempts_ = 4;
  unsigned int max_goal_sampling_attempts_ = 1000;
  unsigned int max_planning_threads_ = 4;
  double max_solution_segment_length_ = 0.0;
  unsigned int minimum_waypoint_count_ = 2;
  double goal_threshold_ = 0.005;

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  context->setMinimumWaypointCount(minimum_waypoint_count_);
  context->setGoalThreshold(goal_threshold_);
}

planning_interface::PlannerConfigurationSettings getPlannerConfigSettings(
    const planning_interface::PlannerConfigurationMap& pconfig_map,
    const std::string& planner_id) {
  auto pc = pconfig_map.end();
  pc = pconfig_map.find(planner_id);
  if (pc == pconfig_map.end()) {
    ROS_ERROR_NAMED(LOGNAME,
                    "Cannot find planning configuration for planner %s ",
                    planner_id.c_str());
    return planning_interface::PlannerConfigurationSettings();
  } else {
    ROS_INFO_NAMED(LOGNAME, "Found planning configuration for planner %s.",
                   planner_id.c_str());
  }
  return pc->second;
}

ompl_interface::ModelBasedPlanningContextPtr createPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const moveit::core::RobotModelPtr& robot_model,
    const moveit_msgs::MotionPlanRequest& req, const ros::NodeHandle& nh) {
  const std::string group_name = "panda_arm";
  ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model,
                                                               group_name);
  ompl_interface::ModelBasedStateSpacePtr state_space =
      std::make_shared<ompl_interface::JointModelStateSpace>(space_spec);
  state_space->computeLocations();

  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface =
      getOMPLInterface(robot_model, nh);

  planning_interface::PlannerConfigurationMap pconfig_map =
      ompl_interface->getPlannerConfigurations();

  planning_interface::PlannerConfigurationSettings pconfig_settings =
      getPlannerConfigSettings(pconfig_map, req.planner_id);

  ompl_interface::PlanningContextManager planning_context_manager =
      ompl_interface->getPlanningContextManager();

  ompl_interface::ModelBasedPlanningContextSpecification context_spec;
  context_spec.config_ = pconfig_settings.config;
  context_spec.planner_selector_ =
      planning_context_manager.getPlannerSelector();
  context_spec.constraint_sampler_manager_ =
      std::make_shared<constraint_samplers::ConstraintSamplerManager>();
  context_spec.state_space_ = state_space;

  ompl::geometric::SimpleSetupPtr ompl_simple_setup =
      std::make_shared<ompl::geometric::SimpleSetup>(state_space);
  context_spec.ompl_simple_setup_ = ompl_simple_setup;

  ompl_interface::ModelBasedPlanningContextPtr context =
      std::make_shared<ompl_interface::ModelBasedPlanningContext>(group_name,
                                                                  context_spec);

  setPlanningContextParams(context);

  context->clear();

  moveit::core::RobotStatePtr start_state =
      planning_scene->getCurrentStateUpdated(req.start_state);

  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);
  context->setCompleteInitialState(*start_state);

  context->setPlanningVolume(req.workspace_parameters);
  moveit_msgs::MoveItErrorCodes error_code;
  if (!context->setPathConstraints(req.path_constraints, &error_code)) {
    ROS_ERROR_NAMED(LOGNAME, "context->setPathConstraints() error: %d",
                    error_code.val);
    return ompl_interface::ModelBasedPlanningContextPtr();
  }

  if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints,
                                   &error_code)) {
    ROS_ERROR_NAMED(LOGNAME, "context->setGoalConstraints() error %d",
                    error_code.val);
    return ompl_interface::ModelBasedPlanningContextPtr();
  }

  bool use_constraints_approximation = true;
  try {
    context->configure(nh, use_constraints_approximation);
    ROS_INFO_NAMED(LOGNAME, "%s: New planning context is set.",
                   context->getName().c_str());
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  } catch (ompl::Exception& ex) {
    ROS_ERROR_NAMED(LOGNAME, "OMPL encountered an error: %s", ex.what());
    context.reset();
  }

  return context;
}

void printStateSpace(
    const ompl_interface::ModelBasedStateSpacePtr& state_space) {
  int type = state_space->getType();
  ROS_INFO_NAMED(LOGNAME, "state type %d", type);
  double measure = state_space->getMeasure();
  ROS_INFO_NAMED(LOGNAME, "state measure %f", measure);
  int dim = state_space->getDimension();
  ROS_INFO_NAMED(LOGNAME, "dimension %d", dim);
  state_space->Diagram(std::cout);
  state_space->List(std::cout);
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Pose& pose) {
  os << "position (x,y,z): " << pose.position.x << ", " << pose.position.y
     << ", " << pose.position.z << "\n";
  os << "orientation (x,y,z,w): " << pose.orientation.x << ", "
     << pose.orientation.y << ", " << pose.orientation.z << ", "
     << pose.orientation.w << "\n";
  return os;
}

Eigen::Vector3d scaleToDist(Eigen::Vector3d vec) {
  double y_max = 1.0;
  double D = 20.0;
  Eigen::Vector3d vec_out = (vec * y_max) / (D * vec.squaredNorm() + 1.0);
  return vec_out;
}

Eigen::VectorXd toEigen(std::vector<double> stl_vec) {  // const
  std::size_t size = stl_vec.size();
  Eigen::VectorXd eig_vec = Eigen::VectorXd::Zero(size);
  for (std::size_t i = 0; i < size; i++) {
    eig_vec[i] = stl_vec[i];
  }
  return eig_vec;
}

std::vector<double> toStlVec(Eigen::VectorXd eig_vec) {  // const
  std::size_t size = eig_vec.size();
  std::vector<double> stl_vec(size, 0.0);
  for (std::size_t i = 0; i < size; i++) {
    stl_vec[i] = eig_vec[i];
  }
  return stl_vec;
}

std::vector<double> toStlVec(
    const ompl::base::RealVectorStateSpace::StateType& vec_state,
    std::size_t size) {  // const
  std::vector<double> stl_vec(size, 0.0);
  for (std::size_t i = 0; i < size; i++) {
    stl_vec[i] = vec_state[i];
  }
  return stl_vec;
}

Eigen::MatrixXd getLinkPositions(moveit::core::RobotStatePtr robot_state) {
  std::vector<std::string> link_names = joint_model_group_->getLinkModelNames();
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  Eigen::MatrixXd positions(dof_, 3);
  for (std::size_t i = 0; i < dof_; i++) {
    const moveit::core::LinkModel* link_model =
        robot_state->getLinkModel(link_names[i]);
    tf = robot_state->getGlobalLinkTransform(link_model);
    auto translation = tf.translation();
    positions(i, 0) = (double)translation.x();
    positions(i, 1) = (double)translation.y();
    positions(i, 2) = (double)translation.z();
  }
  return positions;
}

void saveOriginVec(const Eigen::Vector3d& origin, const Eigen::Vector3d& vec,
                   std::size_t link_num) {
  if (sample_state_count_ >= repulsed_vec_at_link_.size()) {
    repulsed_vec_at_link_.emplace_back(Eigen::VectorXd::Zero(dof_ * 3));
    repulsed_origin_at_link_.emplace_back(Eigen::VectorXd::Zero(dof_ * 3));
  }

  Eigen::VectorXd& cur_repulsed = repulsed_vec_at_link_[sample_state_count_];
  cur_repulsed[link_num * 3] = vec[0];
  cur_repulsed[link_num * 3 + 1] = vec[1];
  cur_repulsed[link_num * 3 + 2] = vec[2];

  Eigen::VectorXd& cur_origin = repulsed_origin_at_link_[sample_state_count_];
  cur_origin[link_num * 3] = origin[0];
  cur_origin[link_num * 3 + 1] = origin[1];
  cur_origin[link_num * 3 + 2] = origin[2];
}

void saveJointAngles(const std::vector<double>& joint_angles) {
  sample_joint_angles_.emplace_back(joint_angles);
}

void saveRepulseAngles(const std::vector<double>& joint_angles,
                       const Eigen::VectorXd& d_q_out) {
  saveJointAngles(joint_angles);
  sample_desired_angles_.emplace_back(
      toStlVec(toEigen(joint_angles) + d_q_out));
}

Eigen::VectorXd obstacleField(const ompl::base::State* base_state) {
  const ompl::base::RealVectorStateSpace::StateType& vec_state =
      *base_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles = toStlVec(vec_state, dof_);

  moveit::core::RobotStatePtr robot_state =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state->setJointGroupPositions(joint_model_group_, joint_angles);
  Eigen::MatrixXd link_positions = getLinkPositions(robot_state);

  std::vector<Eigen::Vector3d> link_to_obs_vec(dof_);
  for (std::size_t i = 0; i < dof_; i++) {
    Eigen::Vector3d origin(link_positions(i, 0), link_positions(i, 1),
                           link_positions(i, 2));
    Eigen::Vector3d vec = origin - obstacle_pos_;
    vec = scaleToDist(vec);
    link_to_obs_vec[i] = vec;

    saveOriginVec(origin, vec, i);
  }

  Eigen::MatrixXcd eigen_values;
  Eigen::MatrixXcd eigen_vectors;
  bool is_found = kinematics_metrics_->getManipulabilityEllipsoid(
      *robot_state, joint_model_group_, eigen_values, eigen_vectors);
  if (!is_found) {
    ROS_INFO_NAMED(LOGNAME, "Unable to get manipulability ellipsoid");
  }

  Eigen::MatrixXd jacobian = robot_state->getJacobian(joint_model_group_);
  // std::cout << "jacobian:\n " << jacobian << std::endl;
  Eigen::VectorXd d_q_out = toEigen(joint_angles);

  for (std::size_t i = 0; i < dof_; i++) {
    Eigen::MatrixXd link_jac = jacobian.block(0, 0, 6, i + 1);
    // std::cout << "link_jac:\n " << link_jac << std::endl;
    Eigen::MatrixXd jac_pinv_;
    pseudoInverse(link_jac, jac_pinv_);
    // std::cout << "jac_pinv_:\n " << jac_pinv_ << std::endl;
    Eigen::Vector3d vec = link_to_obs_vec[i];
    // std::cout << "vec:\n " << vec << std::endl;
    Eigen::VectorXd rob_vec =
        toEigen(std::vector<double>{vec[0], vec[1], vec[2], 0.0, 0.0, 0.0});
    Eigen::VectorXd d_q = jac_pinv_ * rob_vec;
    // std::cout << "d_q:\n " << d_q << std::endl;
    d_q_out[i] = d_q[i];
  }

  saveRepulseAngles(joint_angles, d_q_out);
  sample_state_count_++;
  d_q_out.normalize();
  d_q_out = d_q_out * 0.3;
  return d_q_out;
}

Eigen::VectorXd goalField(const ompl::base::State* state) {
  const ompl::base::RealVectorStateSpace::StateType& x =
      *state->as<ompl::base::RealVectorStateSpace::StateType>();
  Eigen::VectorXd v(7);
  v[0] = joint_goal_pos_[0] - x[0];
  v[1] = joint_goal_pos_[1] - x[1];
  v[2] = joint_goal_pos_[2] - x[2];
  v[3] = joint_goal_pos_[3] - x[3];
  v[4] = joint_goal_pos_[4] - x[4];
  v[5] = joint_goal_pos_[5] - x[5];
  v[6] = joint_goal_pos_[6] - x[6];
  v.normalize();
  return v;
}

Eigen::VectorXd totalField(const ompl::base::State* state) {
  const ompl::base::RealVectorStateSpace::StateType& x =
      *state->as<ompl::base::RealVectorStateSpace::StateType>();
  Eigen::VectorXd goal_vec = goalField(state);
  std::cout << "goal_vec\n" << goal_vec.transpose() << std::endl;
  Eigen::VectorXd obstacle_vec = obstacleField(state);
  std::cout << "obstacle_vec\n" << obstacle_vec.transpose() << std::endl;
  Eigen::VectorXd total_vec = goal_vec + obstacle_vec;
  std::cout << "total_vec\n" << total_vec.transpose() << std::endl;
  total_vec.normalize();
  return total_vec;
}

ompl::base::PlannerPtr createPlanner(
    const ompl::base::SpaceInformationPtr& si) {
  double exploration = 0.7;
  double initial_lambda = 1.0;
  unsigned int update_freq = 100;
  ompl::base::PlannerPtr planner = std::make_shared<ompl::geometric::VFRRT>(
      si, totalField, exploration, initial_lambda, update_freq);
  return planner;
}

void changePlanner(ompl_interface::ModelBasedPlanningContextPtr& context) {
  ompl::geometric::SimpleSetupPtr simple_setup = context->getOMPLSimpleSetup();
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();
  ompl::base::PlannerPtr planner = createPlanner(si);
  simple_setup->setPlanner(planner);
}

bool generatePlan(const ompl_interface::ModelBasedPlanningContextPtr& context,
                  planning_interface::MotionPlanResponse& res) {
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  bool is_solved = context->solve(res);
  if (is_solved) {
    std::size_t state_count = res.trajectory_->getWayPointCount();
    ROS_DEBUG_STREAM("Motion planner reported a solution path with "
                     << state_count << " states");
  }
  ROS_INFO_NAMED(LOGNAME, "Is plan generated? %d", is_solved);
  return is_solved;
}

moveit_msgs::Constraints createPoseGoal() {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(
          "panda_link8", pose, tolerance_pose, tolerance_angle);
  return pose_goal;
}

moveit_msgs::Constraints createJointGoal(
    const moveit::core::RobotStatePtr& robot_state,
    const moveit::core::JointModelGroup* joint_model_group) {
  moveit::core::RobotState goal_state(*robot_state);
  goal_state.setJointGroupPositions(joint_model_group, joint_goal_pos_);
  double tolerance = 0.001;
  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(
          goal_state, joint_model_group, tolerance);
  return joint_goal;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_and_place_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  ROS_DEBUG_NAMED(LOGNAME, "Start!");

  // Initialization
  // ^^^^^^^^^^^^^^
  // ^^^^^^^^^^^^^^
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      "robot_description");

  robot_model_ = robot_model_loader_->getModel();

  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      robot_model_loader_);

  /* listen for planning scene messages on topic /XXX and apply them to
                       the internal planning scene accordingly */
  psm_->startSceneMonitor();
  /* listens to changes of world geometry, collision objects, and (optionally)
   * octomaps */
  psm_->startWorldGeometryMonitor();
  /* listen to joint state updates as well as changes in attached collision
     objects and update the internal planning scene accordingly*/
  psm_->startStateMonitor();

  kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getRobotModel());

  /* We can get the most up to date robot state from the PlanningSceneMonitor
     by locking the internal planning scene for reading. This lock ensures
     that the underlying scene isn't updated while we are reading it's state.
     RobotState's are useful for computing the forward and inverse kinematics
     of the robot among many other uses */
  planning_scene_monitor::CurrentStateMonitorPtr csm = psm_->getStateMonitor();
  csm->enableCopyDynamics(true);

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));

  /* Create a JointModelGroup to keep track of the current robot pose and
     planning group. The Joint Model group is useful for dealing with one set
     of joints at a time such as a left arm or a end effector */
  joint_model_group_ = robot_model_->getJointModelGroup(group_name_);

  robot_state->setToDefaultValues(joint_model_group_, "ready");
  psm_->updateSceneWithCurrentState();

  ROS_INFO_NAMED(LOGNAME, "Robot State Positions");
  robot_state->printStatePositions();
  robot_state_ = std::make_shared<moveit::core::RobotState>(*robot_state);

  marker_pub_ = node_handle.advertise<visualization_msgs::MarkerArray>(
      "robot_state", 1, true);
  arrow_pub_ = node_handle.advertise<visualization_msgs::MarkerArray>(
      "vector_field", 1, true);
  trajectory_publisher_ = node_handle.advertise<moveit_msgs::DisplayTrajectory>(
      "planned_path", 1, true);
  rep_state_publisher_ = node_handle.advertise<moveit_msgs::DisplayTrajectory>(
      "repulsed_state", 1, true);

  // namespace rvt = rviz_visual_tools;
  visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, robot_model_);

  // We can now setup the PlanningPipeline object, which will use the ROS
  // parameter server to determine the set of request adapters and the
  // planning plugin to use
  // planning_pipeline::PlanningPipelinePtr planning_pipeline(
  //     new planning_pipeline::PlanningPipeline(
  //         robot_model_loader_->getModel(), node_handle, "planning_plugin",
  //         "request_adapters"));

  // const planning_interface::PlannerManagerPtr planner_manager =
  //     planning_pipeline->getPlannerManager();

  // Pose Goal
  // ^^^^^^^^^
  // ^^^^^^^^^
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  setCurToStartState(req, robot_state, joint_model_group_);

  req.goal_constraints.clear();
  moveit_msgs::Constraints goal =
      createJointGoal(robot_state, joint_model_group_);
  req.goal_constraints.push_back(goal);

  req.group_name = group_name_;
  req.allowed_planning_time = 1.0;
  req.planner_id = "panda_arm[RRT]";

  // Before planning, we will need a Read Only lock on the planning scene so
  // that it does not modify the world representation while planning
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);

  ompl_interface::ModelBasedPlanningContextPtr context =
      createPlanningContext(lscene, robot_model_, req, node_handle);

  changePlanner(context);

  generatePlan(context, res);

  // /* Now, call the pipeline and check whether planning was successful. */
  // planning_pipeline->generatePlan(lscene, req, res);

  // /* Now, call the pipeline and check whether planning was successful. */
  // /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully. Error code: %d",
              res.error_code_.val);
    // return 0;
  }
  visualizeRepulsedState();

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  // ^^^^^^^^^^^^^^^^^^^^

  visualizeTrajectory(res);

  // Execute Trajectory
  // ^^^^^^^^^^^^^^^^^^^^
  // ^^^^^^^^^^^^^^^^^^^^
  promptAnyInput();

  std::cout << "Finished!" << std::endl;

  return 0;
}
