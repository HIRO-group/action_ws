#include "contact_planner.h"

constexpr char LOGNAME[] = "contact_planner";

namespace pick_and_place {

const std::vector<double> ContactPlanner::joint_goal_pos_{-1.0, 0.7, 0.7, -1.0,
                                                          -0.7, 2.0, 0.0};

void ContactPlanner::pseudoInverse(const Eigen::MatrixXd& M_,
                                   Eigen::MatrixXd& M_pinv_, bool damped) {
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

void ContactPlanner::printPlannerConfigMap(
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

void ContactPlanner::printJointTrajectory(
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

void ContactPlanner::visualizeManipVec(std::size_t state_num) {
  visualization_msgs::MarkerArray marker_array;

  if (repulsed_origin_at_link_.size() <= state_num ||
      manipulability_.size() <= state_num) {
    ROS_DEBUG_NAMED(
        LOGNAME,
        "Insufficient data stored to vizualize manipulability vectors.");
    return;
  }

  auto repulsed_origin_at_link = repulsed_origin_at_link_[state_num];
  std::vector<Manipulability> manip_per_joint = manipulability_[state_num];

  for (std::size_t link_num = 0; link_num < dof_; link_num++) {
    Eigen::Vector3d origin(repulsed_origin_at_link[link_num * 3],
                           repulsed_origin_at_link[link_num * 3 + 1],
                           repulsed_origin_at_link[link_num * 3 + 2]);
    Manipulability manip = manip_per_joint[link_num];

    for (std::size_t vec_num = 0; vec_num < manip.eigen_values.size();
         vec_num++) {
      Eigen::Vector3d dir =
          manip.getVector(vec_num) * manip.eigen_values(vec_num) * 0.2;
      dir = dir;
      Eigen::Vector3d vec = origin + dir;

      uint32_t shape = visualization_msgs::Marker::ARROW;

      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      // marker.ns = "basic_shapes";
      marker.id = link_num * 3 + vec_num;
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

      if (manip.pass) {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
      } else {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
      }

      marker.lifetime = ros::Duration();
      marker_array.markers.push_back(marker);
    }
  }
  manipulability_pub_.publish(marker_array);
}

void ContactPlanner::visualizeRepulseVec(std::size_t state_num) {
  if (repulsed_origin_at_link_.size() <= state_num ||
      repulsed_vec_at_link_.size() <= state_num) {
    ROS_DEBUG_NAMED(LOGNAME,
                    "Insufficient data stored to vizualize repulsive vector.");
    return;
  }

  visualization_msgs::MarkerArray marker_array;
  auto repulsed_origin_at_link = repulsed_origin_at_link_[state_num];
  auto repulsed_vec_at_link = repulsed_vec_at_link_[state_num];

  // std::cout << "repulsed_origin_at_link.size(): "
  //           << repulsed_origin_at_link.size() << std::endl;

  std::size_t max_num = (int)(repulsed_origin_at_link.size() / 3);
  for (std::size_t i = 0; i < max_num; i++) {
    // std::cout << "i: " << i << std::endl;
    Eigen::Vector3d origin(repulsed_origin_at_link[i * 3],
                           repulsed_origin_at_link[i * 3 + 1],
                           repulsed_origin_at_link[i * 3 + 2]);

    Eigen::Vector3d dir(repulsed_vec_at_link[i * 3],
                        repulsed_vec_at_link[i * 3 + 1],
                        repulsed_vec_at_link[i * 3 + 2]);

    Eigen::Vector3d vec = origin + dir;

    uint32_t shape = visualization_msgs::Marker::ARROW;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    // marker.ns = "basic_shapes";
    marker.id = i;
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

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker_array.markers.push_back(marker);
  }
  arrow_pub_.publish(marker_array);
}

visualization_msgs::Marker ContactPlanner::getObstacleMarker() {
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

void ContactPlanner::visualizeRepulseOrigin(std::size_t state_num) {
  if (sample_joint_angles_.size() <= state_num) {
    ROS_DEBUG_NAMED(LOGNAME,
                    "Insufficient data stored to vizualize repulsion origin.");
    return;
  }

  visualization_msgs::MarkerArray marker_array;
  auto repulsed_origin_at_link = repulsed_origin_at_link_[state_num];
  std::size_t max_num = (int)(repulsed_origin_at_link.size() / 3);

  for (std::size_t i = 0; i < max_num; i++) {
    // std::cout << "translation for " << link_names[i] << "\n"
    //           << translation << std::endl;

    Eigen::Vector3d origin(repulsed_origin_at_link[i * 3],
                           repulsed_origin_at_link[i * 3 + 1],
                           repulsed_origin_at_link[i * 3 + 2]);

    uint32_t shape = visualization_msgs::Marker::SPHERE;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    // marker.ns = "basic_shapes";
    marker.id = i;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and DELETEALL
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = origin[0];
    marker.pose.position.y = origin[1];
    marker.pose.position.z = origin[2];
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
  robot_marker_pub_.publish(marker_array);
}

void ContactPlanner::visualizeObstacleMarker() {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker = getObstacleMarker();
  marker_array.markers.push_back(marker);
  obstacle_marker_pub_.publish(marker_array);
}

void ContactPlanner::visualizeTrajectory(
    const planning_interface::MotionPlanResponse& res, std::string name) {
  // first publish the final path
  moveit_msgs::DisplayTrajectory display_final_traj;
  moveit_msgs::MotionPlanResponse resp_final_traj;
  ros::Publisher pub_final_traj;

  pub_final_traj = nh_.advertise<moveit_msgs::DisplayTrajectory>(name, 1, true);
  res.getMessage(resp_final_traj);
  display_final_traj.trajectory_start = resp_final_traj.trajectory_start;
  display_final_traj.trajectory.push_back(resp_final_traj.trajectory);
  pub_final_traj.publish(display_final_traj);
  trajectory_publishers_.push_back(pub_final_traj);

  moveit_msgs::DisplayTrajectory display_raw_traj;
  moveit_msgs::MotionPlanResponse resp_raw_traj;
  ros::Publisher pub_raw_traj;

  // next publish the raw path aka the path without smoothing
  pub_raw_traj =
      nh_.advertise<moveit_msgs::DisplayTrajectory>(name + "_raw", 1, true);
  robot_trajectory::RobotTrajectory trajectory = context_->getRawTrajectory();
  trajectory.getRobotTrajectoryMsg(resp_raw_traj.trajectory);

  display_raw_traj.trajectory_start = resp_raw_traj.trajectory_start;
  display_raw_traj.trajectory.push_back(resp_raw_traj.trajectory);
  pub_raw_traj.publish(display_raw_traj);
  trajectory_publishers_.push_back(pub_raw_traj);
}

void ContactPlanner::visualizeTreeStates() {
  ompl::geometric::SimpleSetupPtr simple_setup = context_->getOMPLSimpleSetup();
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();
  ompl::base::PlannerPtr planner = simple_setup->getPlanner();

  ompl::base::PlannerData planner_data(si);
  planner->getPlannerData(planner_data);

  ompl::base::StateStoragePtr storage = planner_data.extractStateStorage();
  std::size_t num_states = storage->size();
  std::cout << "num_states: " << num_states << std::endl;
  std::vector<const ompl::base::State*> states = storage->getStates();

  std::vector<std::vector<double>> joint_states;
  for (std::size_t i = 0; i < num_states; i++) {
    const ompl::base::State* state = states[i];
    const ompl::base::RealVectorStateSpace::StateType& vec_state =
        *state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> joint_angles = toStlVec(vec_state, dof_);
    joint_states.emplace_back(joint_angles);
  }

  const std::size_t num_display_states = joint_states.size();
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;

  sensor_msgs::JointState joint_start_state;
  std::vector<std::string> names =
      joint_model_group_->getActiveJointModelNames();

  std::size_t num_joints = names.size();
  std::vector<double> joint_angles = joint_states[0];
  std::cout << toEigen(joint_angles).transpose() << std::endl;
  joint_start_state.name = names;
  joint_start_state.position = joint_angles;
  joint_start_state.velocity = std::vector<double>(num_joints, 0.0);
  joint_start_state.effort = std::vector<double>(num_joints, 0.0);

  moveit_msgs::RobotState robot_start_state;
  robot_start_state.joint_state = joint_start_state;

  response.group_name = group_name_;
  response.trajectory_start = robot_start_state;
  display_trajectory.trajectory_start = response.trajectory_start;

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = names;

  for (std::size_t i = 0; i < num_display_states; i++) {
    trajectory_msgs::JointTrajectoryPoint point;
    joint_angles = joint_states[i];

    point.positions = joint_angles;
    point.velocities = std::vector<double>(num_joints, 0.0);
    point.accelerations = std::vector<double>(num_joints, 0.0);
    point.effort = std::vector<double>(num_joints, 0.0);
    point.time_from_start = ros::Duration(1.5);
    joint_trajectory.points.push_back(point);
  }
  moveit_msgs::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory = joint_trajectory;
  response.trajectory = robot_trajectory;
  display_trajectory.trajectory.push_back(response.trajectory);
  tree_states_publisher_.publish(display_trajectory);
}

void ContactPlanner::visualizeGoalState() {
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;

  sensor_msgs::JointState joint_start_state;
  std::vector<std::string> names =
      joint_model_group_->getActiveJointModelNames();

  std::size_t num_joints = names.size();
  std::vector<double> joint_angles = joint_goal_pos_;
  std::cout << toEigen(joint_angles).transpose() << std::endl;
  joint_start_state.name = names;
  joint_start_state.position = joint_angles;
  joint_start_state.velocity = std::vector<double>(num_joints, 0.0);
  joint_start_state.effort = std::vector<double>(num_joints, 0.0);

  moveit_msgs::RobotState robot_start_state;
  robot_start_state.joint_state = joint_start_state;

  response.group_name = group_name_;
  response.trajectory_start = robot_start_state;
  display_trajectory.trajectory_start = response.trajectory_start;

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = names;

  trajectory_msgs::JointTrajectoryPoint point;
  joint_angles = joint_goal_pos_;

  point.positions = joint_angles;
  point.velocities = std::vector<double>(num_joints, 0.0);
  point.accelerations = std::vector<double>(num_joints, 0.0);
  point.effort = std::vector<double>(num_joints, 0.0);
  point.time_from_start = ros::Duration(1.5);
  joint_trajectory.points.push_back(point);

  moveit_msgs::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory = joint_trajectory;
  response.trajectory = robot_trajectory;
  display_trajectory.trajectory.push_back(response.trajectory);
  goal_state_publisher_.publish(display_trajectory);
}

void ContactPlanner::visualizeRepulsedState() {
  std::string user_input = " ";

  while (user_input != "q") {
    if (sample_joint_angles_.size() <= viz_state_idx_ ||
        sample_desired_angles_.size() <= viz_state_idx_) {
      ROS_DEBUG_NAMED(LOGNAME,
                      "Insufficient data stored to vizualize repulsed states.");
      return;
    }

    std::cout << "Displaying repulsed state with idx: " << viz_state_idx_
              << std::endl;

    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::MotionPlanResponse response;

    sensor_msgs::JointState joint_start_state;
    std::vector<std::string> names =
        joint_model_group_->getActiveJointModelNames();

    std::size_t num_joints = names.size();
    std::vector<double> joint_angles = sample_joint_angles_[viz_state_idx_];
    std::cout << toEigen(joint_angles).transpose() << std::endl;
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
    std::cout << toEigen(joint_angles).transpose() << std::endl;

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
    visualizeManipVec(viz_state_idx_);

    viz_state_idx_++;

    std::cout << "Press 'q' to exit this visualization or 'c' to go to the "
                 "next state "
              << std::endl;
    std::cin >> user_input;
  }
}

void ContactPlanner::promptAnyInput() {
  std::string user_input = " ";
  std::cout << "Press any key to continue..." << std::endl;
  std::cin >> user_input;
}

void ContactPlanner::setCurToStartState(
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

std::unique_ptr<ompl_interface::OMPLInterface> ContactPlanner::getOMPLInterface(
    const moveit::core::RobotModelConstPtr& model, const ros::NodeHandle& nh) {
  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface =
      std::make_unique<ompl_interface::OMPLInterface>(model, nh);
  return ompl_interface;
}

void ContactPlanner::setPlanningContextParams(
    ompl_interface::ModelBasedPlanningContextPtr& context) {
  unsigned int max_goal_samples_ = 10;
  unsigned int max_state_sampling_attempts_ = 4;
  unsigned int max_goal_sampling_attempts_ = 1000;
  unsigned int max_planning_threads_ = 4;
  double max_solution_segment_length_ = 0.0;
  unsigned int minimum_waypoint_count_ = 2;
  double goal_threshold_ = 0.1;
  bool simplify_solution_ = true;
  bool interpolate_ = true;
  bool hybridize_ = false;

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  context->setMinimumWaypointCount(minimum_waypoint_count_);
  context->setGoalThreshold(goal_threshold_);
  context->simplifySolutions(simplify_solution_);
  context->setInterpolation(interpolate_);
  context->setHybridize(hybridize_);
}

planning_interface::PlannerConfigurationSettings
ContactPlanner::getPlannerConfigSettings(
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

void ContactPlanner::createPlanningContext(
    const moveit_msgs::MotionPlanRequest& req, const ros::NodeHandle& nh) {
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);

  ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model_,
                                                               group_name_);
  ompl_interface::ModelBasedStateSpacePtr state_space =
      std::make_shared<ompl_interface::JointModelStateSpace>(space_spec);
  state_space->computeLocations();

  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface =
      getOMPLInterface(robot_model_, nh);

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

  context_ = std::make_shared<ompl_interface::ModelBasedPlanningContext>(
      group_name_, context_spec);

  setPlanningContextParams(context_);

  context_->clear();

  moveit::core::RobotStatePtr start_state =
      lscene->getCurrentStateUpdated(req.start_state);

  context_->setPlanningScene(lscene);
  context_->setMotionPlanRequest(req);
  context_->setCompleteInitialState(*start_state);

  context_->setPlanningVolume(req.workspace_parameters);
  moveit_msgs::MoveItErrorCodes error_code;
  if (!context_->setPathConstraints(req.path_constraints, &error_code)) {
    ROS_ERROR_NAMED(LOGNAME, "context_->setPathConstraints() error: %d",
                    error_code.val);
    context_ = ompl_interface::ModelBasedPlanningContextPtr();
  }

  if (!context_->setGoalConstraints(req.goal_constraints, req.path_constraints,
                                    &error_code)) {
    ROS_ERROR_NAMED(LOGNAME, "context_->setGoalConstraints() error %d",
                    error_code.val);
    context_ = ompl_interface::ModelBasedPlanningContextPtr();
  }

  bool use_constraints_approximation = true;
  try {
    context_->configure(nh, use_constraints_approximation);
    ROS_INFO_NAMED(LOGNAME, "%s: New planning context_ is set.",
                   context_->getName().c_str());
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  } catch (ompl::Exception& ex) {
    ROS_ERROR_NAMED(LOGNAME, "OMPL encountered an error: %s", ex.what());
    context_.reset();
  }
}

ompl_interface::ModelBasedPlanningContextPtr
ContactPlanner::getPlanningContext() {
  return context_;
}

moveit_msgs::Constraints ContactPlanner::createJointGoal() {
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

void ContactPlanner::printStateSpace(
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

Eigen::Vector3d ContactPlanner::scaleToDist(Eigen::Vector3d vec) {
  double y_max = 1.0;
  double D = 10.0;
  double dist_max = 0.5;
  // std::cout << "vec.norm() " << vec.norm() << std::endl;
  Eigen::Vector3d vec_out = Eigen::VectorXd::Zero(3);

  // if (vec.norm() > 0.4) {
  //   return vec_out;
  // }

  vec_out = (vec * y_max) / (D * vec.squaredNorm() + 1.0);

  // std::cout << "vec.squaredNorm() " << vec.squaredNorm() << std::endl;
  // std::cout << "before scale " << vec.transpose() << std::endl;
  // std::cout << "vec_out " << vec_out.transpose() << std::endl;
  return vec_out;
}

Eigen::VectorXd ContactPlanner::toEigen(std::vector<double> stl_vec) {  // const
  std::size_t size = stl_vec.size();
  Eigen::VectorXd eig_vec = Eigen::VectorXd::Zero(size);
  for (std::size_t i = 0; i < size; i++) {
    eig_vec[i] = stl_vec[i];
  }
  return eig_vec;
}

std::vector<double> ContactPlanner::toStlVec(
    Eigen::VectorXd eig_vec) {  // const
  std::size_t size = eig_vec.size();
  std::vector<double> stl_vec(size, 0.0);
  for (std::size_t i = 0; i < size; i++) {
    stl_vec[i] = eig_vec[i];
  }
  return stl_vec;
}

std::vector<double> ContactPlanner::toStlVec(
    const ompl::base::RealVectorStateSpace::StateType& vec_state,
    std::size_t size) {  // const
  std::vector<double> stl_vec(size, 0.0);
  for (std::size_t i = 0; i < size; i++) {
    stl_vec[i] = vec_state[i];
  }
  return stl_vec;
}

Eigen::MatrixXd ContactPlanner::getLinkPositions(
    moveit::core::RobotStatePtr robot_state) {
  std::vector<std::string> link_names = joint_model_group_->getLinkModelNames();
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  Eigen::MatrixXd positions(link_names.size(), 3);
  for (std::size_t i = 0; i < link_names.size(); i++) {
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

void ContactPlanner::saveOriginVec(const Eigen::Vector3d& origin,
                                   const Eigen::Vector3d& vec,
                                   std::size_t num_pts, std::size_t pt_num) {
  if (sample_state_count_ >= repulsed_vec_at_link_.size()) {
    repulsed_vec_at_link_.emplace_back(Eigen::VectorXd::Zero(num_pts * 3));
    repulsed_origin_at_link_.emplace_back(Eigen::VectorXd::Zero(num_pts * 3));
  }

  Eigen::VectorXd& cur_repulsed = repulsed_vec_at_link_[sample_state_count_];
  cur_repulsed[pt_num * 3] = vec[0];
  cur_repulsed[pt_num * 3 + 1] = vec[1];
  cur_repulsed[pt_num * 3 + 2] = vec[2];

  Eigen::VectorXd& cur_origin = repulsed_origin_at_link_[sample_state_count_];
  cur_origin[pt_num * 3] = origin[0];
  cur_origin[pt_num * 3 + 1] = origin[1];
  cur_origin[pt_num * 3 + 2] = origin[2];
}

void ContactPlanner::saveJointAngles(const std::vector<double>& joint_angles) {
  sample_joint_angles_.emplace_back(joint_angles);
}

void ContactPlanner::saveRepulseAngles(const std::vector<double>& joint_angles,
                                       const Eigen::VectorXd& d_q_out) {
  saveJointAngles(joint_angles);
  sample_desired_angles_.emplace_back(
      toStlVec(toEigen(joint_angles) + d_q_out));
}

double ContactPlanner::getDistance(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2) +
              pow(p2[2] - p1[2], 2));
}

void ContactPlanner::interpolateLinkPositions(Eigen::MatrixXd& mat) {
  std::size_t i = 1;
  while (i < mat.rows() - 1) {
    // std::cout << "i: " << i << std::endl;
    // std::cout << "mat.rows(): " << mat.rows() << std::endl;
    // std::cout << "mat.cols(): " << mat.cols() << std::endl;
    // std::cout << "mat\n: " << mat << std::endl;

    Eigen::Vector3d p1(mat(i - 1, 0), mat(i - 1, 1), mat(i - 1, 2));
    Eigen::Vector3d p2(mat(i, 0), mat(i, 1), mat(i, 2));
    double dist = getDistance(p1, p2);
    // std::cout << "p1: " << p1.transpose() << std::endl;
    // std::cout << "p2: " << p2.transpose() << std::endl;
    // std::cout << "dist: " << dist << std::endl;

    if (dist > 0.05) {
      Eigen::MatrixXd imat = Eigen::MatrixXd::Zero(mat.rows() + 1, 3);
      imat.topRows(i) = mat.topRows(i);
      imat.bottomRows(mat.rows() - i) = mat.bottomRows(mat.rows() - i);
      Eigen::Vector3d ivec((p2[0] + p1[0]) / 2.0, (p2[1] + p1[1]) / 2.0,
                           (p2[2] + p1[2]) / 2.0);
      // std::cout << "ivec: " << ivec.transpose() << std::endl;

      imat(i, 0) = ivec[0];
      imat(i, 1) = ivec[1];
      imat(i, 2) = ivec[2];
      // std::cout << "mat\n: " << mat << std::endl;
      // std::cout << "imat\n: " << imat << std::endl;
      mat = imat;
    } else {
      i++;
    }
  }
}

Eigen::VectorXd ContactPlanner::obstacleField(
    const ompl::base::State* base_state) {
  const ompl::base::RealVectorStateSpace::StateType& vec_state =
      *base_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles = toStlVec(vec_state, dof_);

  moveit::core::RobotStatePtr robot_state =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state->setJointGroupPositions(joint_model_group_, joint_angles);
  Eigen::MatrixXd rob_pts = getLinkPositions(robot_state);

  interpolateLinkPositions(rob_pts);
  // std::cout << "rob_pts\n: " << rob_pts << std::endl;
  std::size_t num_rob_pts = rob_pts.rows();
  // std::cout << "num_rob_pts: " << num_rob_pts << std::endl;

  std::vector<double> dist_to_obs(num_rob_pts);
  for (std::size_t i = 0; i < num_rob_pts; i++) {
    Eigen::Vector3d origin(rob_pts(i, 0), rob_pts(i, 1), rob_pts(i, 2));
    Eigen::Vector3d vec = origin - obstacle_pos_;
    double dist = vec.norm();
    dist_to_obs[i] = dist;
  }

  auto min_itr =
      std::min_element(std::begin(dist_to_obs), std::end(dist_to_obs));
  std::size_t loc_min = std::distance(std::begin(dist_to_obs), min_itr);
  // std::cout << "loc_min " << loc_min << std::endl;

  std::vector<Eigen::Vector3d> link_to_obs_vec(num_rob_pts,
                                               Eigen::Vector3d::Zero(3));
  for (std::size_t i = 0; i < num_rob_pts; i++) {
    Eigen::Vector3d origin(rob_pts(i, 0), rob_pts(i, 1), rob_pts(i, 2));
    Eigen::Vector3d vec = Eigen::Vector3d::Zero(3);
    vec = origin - obstacle_pos_;
    vec = scaleToDist(vec);
    link_to_obs_vec[i] = vec;
    saveOriginVec(origin, vec, num_rob_pts, i);
  }
  // std::cout << "link_to_obs_vec.size() " << link_to_obs_vec.size() <<
  // std::endl;

  Eigen::MatrixXd jacobian = robot_state->getJacobian(joint_model_group_);

  // std::vector<Manipulability> manip_per_joint;

  // std::cout << "jacobian:\n " << jacobian << std::endl;
  // Eigen::VectorXd d_q_out = toEigen(joint_angles);
  Eigen::VectorXd d_q_out = Eigen::VectorXd::Zero(dof_);
  for (std::size_t i = 0; i < dof_; i++) {
    Eigen::MatrixXd link_jac = jacobian.block(0, 0, 3, i + 1);
    // std::cout << "link_jac:\n " << link_jac << std::endl;

    Eigen::MatrixXd jac_pinv_;
    pseudoInverse(link_jac, jac_pinv_);
    // std::cout << "jac_pinv_:\n " << jac_pinv_ << std::endl;
    Eigen::Vector3d vec = link_to_obs_vec[i];
    // std::cout << "vec:\n " << vec << std::endl;
    Eigen::Vector3d rob_vec =
        toEigen(std::vector<double>{vec[0], vec[1], vec[2]});

    Eigen::MatrixXd j_t = link_jac.transpose();
    Eigen::MatrixXd j_sq = link_jac * j_t;

    // Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(j_sq);
    // Manipulability manip;
    // manip.eigen_values = eigensolver.eigenvalues().real();
    // manip.eigen_vectors = eigensolver.eigenvectors().real();

    // bool is_valid = false;
    // for (std::size_t i = 0; i < manip.eigen_values.size(); i++) {
    //   double eig_val = manip.eigen_values(i);
    //   // std::cout << "eig_val " << eig_val << std::endl;
    //   Eigen::Vector3d eig_vec = manip.getVector(i);
    //   // double ab = eig_vec.dot(rob_vec);
    //   // double aa = eig_vec.dot(eig_vec);
    //   // double bb = rob_vec.dot(rob_vec);

    //   double sim1 = std::pow(eig_vec.dot(rob_vec), 2) /
    //                 (eig_vec.dot(eig_vec) * rob_vec.dot(rob_vec));
    //   double sim2 = std::pow(eig_vec.dot(-1 * rob_vec), 2) /
    //                 (eig_vec.dot(eig_vec) * -1 * rob_vec.dot(-1 * rob_vec));
    //   double sim = std::max(sim1, sim2);
    //   std::cout << "eig_vec " << eig_vec.transpose() << std::endl;
    //   std::cout << "rob_vec " << rob_vec.transpose() << std::endl;
    //   std::cout << "sim " << sim << std::endl;

    //   const double VAL_THRESH = 0.2;
    //   const double SIM_THRESH = 0.2;
    //   if (eig_val > VAL_THRESH && sim > SIM_THRESH) {
    //     std::cout << "thresh passed" << std::endl;
    //     is_valid = true;
    //   }
    // }

    // is_valid = true;
    // manip.pass = is_valid;
    // manip_per_joint.emplace_back(manip);

    double repulse_angle = 0.0;
    // if (is_valid) {
    Eigen::VectorXd d_q = jac_pinv_ * rob_vec;
    // std::cout << "d_q:\n " << d_q << std::endl;
    repulse_angle = d_q[i];
    // }

    d_q_out[i] = repulse_angle;
  }

  // d_q_out = d_q_out * 0.5;
  // d_q_out.normalize();
  // std::cout << "d_q_out:\n " << d_q_out.transpose() << std::endl;

  // manipulability_.emplace_back(manip_per_joint);
  saveRepulseAngles(joint_angles, d_q_out);
  sample_state_count_++;

  return d_q_out;
}

Eigen::VectorXd ContactPlanner::goalField(const ompl::base::State* state) {
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

Eigen::VectorXd ContactPlanner::negGoalField(const ompl::base::State* state) {
  const ompl::base::RealVectorStateSpace::StateType& x =
      *state->as<ompl::base::RealVectorStateSpace::StateType>();
  Eigen::VectorXd v(7);
  v[0] = x[0] - joint_goal_pos_[0];
  v[1] = x[1] - joint_goal_pos_[1];
  v[2] = x[2] - joint_goal_pos_[2];
  v[3] = x[3] - joint_goal_pos_[3];
  v[4] = x[4] - joint_goal_pos_[4];
  v[5] = x[5] - joint_goal_pos_[5];
  v[6] = x[6] - joint_goal_pos_[6];
  v.normalize();
  return v;
}

Eigen::VectorXd ContactPlanner::totalField(const ompl::base::State* state) {
  const ompl::base::RealVectorStateSpace::StateType& x =
      *state->as<ompl::base::RealVectorStateSpace::StateType>();
  Eigen::VectorXd goal_vec = goalField(state);
  std::cout << "\ngoal_vec\n" << goal_vec.transpose() << std::endl;
  Eigen::VectorXd obstacle_vec = obstacleField(state);
  std::cout << "obstacle_vec\n" << obstacle_vec.transpose() << std::endl;
  Eigen::VectorXd total_vec = goal_vec + obstacle_vec;
  std::cout << "total_vec\n" << total_vec.transpose() << std::endl;
  total_vec.normalize();
  return total_vec;
}

void ContactPlanner::changePlanner() {
  ompl::geometric::SimpleSetupPtr simple_setup = context_->getOMPLSimpleSetup();
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();

  ompl::geometric::PathSimplifierPtr simplifier =
      simple_setup->getPathSimplifier();

  simplifier->setSimplificationType(
      ompl::geometric::SimplificationType::SMOOTH_COST);

  std::function<Eigen::VectorXd(const ompl::base::State*)> vFieldFunc =
      std::bind(&ContactPlanner::obstacleField, this, std::placeholders::_1);

  // double exploration = 0.5;
  // double initial_lambda = 0.01;
  // unsigned int update_freq = 30;
  // ompl::base::PlannerPtr planner = std::make_shared<ompl::geometric::CVFRRT>(
  //     si, vFieldFunc, exploration, initial_lambda, update_freq);

  simple_setup->setOptimizationObjective(
      std::make_shared<ompl::base::VFUpstreamCriterionOptimizationObjective>(
          si, vFieldFunc));

  ompl::base::PlannerPtr planner =
      std::make_shared<ompl::geometric::ClassicTRRT>(
          simple_setup->getSpaceInformation());

  simple_setup->setPlanner(planner);
}

bool ContactPlanner::generatePlan(planning_interface::MotionPlanResponse& res) {
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  bool is_solved = context_->solve(res);
  if (is_solved) {
    std::size_t state_count = res.trajectory_->getWayPointCount();
    ROS_DEBUG_STREAM("Motion planner reported a solution path with "
                     << state_count << " states");
  }
  ROS_INFO_NAMED(LOGNAME, "Is plan generated? %d", is_solved);
  return is_solved;
}

moveit_msgs::Constraints ContactPlanner::createPoseGoal() {
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

void ContactPlanner::init() {
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      "robot_description");

  robot_model_ = robot_model_loader_->getModel();

  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      robot_model_loader_);

  psm_->startSceneMonitor();

  psm_->startWorldGeometryMonitor();

  psm_->startStateMonitor();

  psm_->startWorldGeometryMonitor();

  kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getRobotModel());

  planning_scene_monitor::CurrentStateMonitorPtr csm = psm_->getStateMonitor();
  csm->enableCopyDynamics(true);

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));

  joint_model_group_ = robot_model_->getJointModelGroup(group_name_);

  robot_state->setToDefaultValues(joint_model_group_, "ready");
  psm_->updateSceneWithCurrentState();

  ROS_INFO_NAMED(LOGNAME, "Robot State Positions");
  robot_state->printStatePositions();
  robot_state_ = std::make_shared<moveit::core::RobotState>(*robot_state);

  robot_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("repulse_origin", 1, true);
  obstacle_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("obstacle", 1, true);
  arrow_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vector_field", 1, true);
  manipulability_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("manipulability", 1, true);
  rep_state_publisher_ =
      nh_.advertise<moveit_msgs::DisplayTrajectory>("repulsed_state", 1, true);
  tree_states_publisher_ =
      nh_.advertise<moveit_msgs::DisplayTrajectory>("tree_states", 1, true);
  goal_state_publisher_ =
      nh_.advertise<moveit_msgs::DisplayTrajectory>("goal_state", 1, true);

  // namespace rvt = rviz_visual_tools;
  visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, robot_model_);

  visualizeObstacleMarker();
  visualizeGoalState();
}

std::string ContactPlanner::getGroupName() { return group_name_; }

}  // namespace pick_and_place